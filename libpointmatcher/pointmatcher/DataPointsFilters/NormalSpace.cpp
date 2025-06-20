// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "NormalSpace.h"

#include <algorithm>
#include <vector>
#include <random>
#include <ciso646>
#include <cmath>
#include <numeric>

// NormalSpaceDataPointsFilter
template <typename T>
NormalSpaceDataPointsFilter<T>::NormalSpaceDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("NormalSpaceDataPointsFilter",
		NormalSpaceDataPointsFilter::availableParameters(), params),
	nbSample{Parametrizable::get<std::size_t>("nbSample")},
	seed{Parametrizable::get<std::size_t>("seed")},
	epsilon{Parametrizable::get<T>("epsilon")},
	randomShuffle{Parametrizable::get<bool>("randomShuffle")},
	nbBucket{std::size_t(ceil(2.0 * M_PI / epsilon) * ceil(M_PI / epsilon))}
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
NormalSpaceDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

//TODO: Add support for 2D by building histogram of polar coordinate with uniform sampling

template <typename T>
void NormalSpaceDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	//check dimension
	const Index featDim{cloud.features.rows()};
	if(featDim < 4) //3D case support only
	{
		std::cerr << "ERROR: NormalSpaceDataPointsFilter does not support 2D point cloud yet (does nothing)" << std::endl;
		return;
	}

	//Check number of points
	const Index nbPoints{cloud.getNbPoints()};
	if(nbSample >= std::size_t(nbPoints))
		return;

	//Check if there is normals info
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");

	const auto& normals = cloud.getDescriptorViewByName("normals");

	std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with seed

	//bucketed normal space
	std::vector<std::vector<int>> idBuckets;
	idBuckets.resize(nbBucket);

	std::vector<std::size_t> keepIndexes;
	keepIndexes.reserve(nbSample);

	// Generate a random sequence of indices so that elements are placed in buckets
	std::vector<std::size_t> pointIndices(nbPoints);
	std::iota(pointIndices.begin(), pointIndices.end(), 0); // 0-nbPoints
	
	// Points are shuffled randomly to ensure a uniform distribution of normals in the sampling set.
	if(randomShuffle)
	{
		std::random_shuffle(pointIndices.begin(), pointIndices.end());
	}

	// We compute the spherical coordinate of the point cloud points based on their normal vectors.
	// Reference: http://corysimon.github.io/articles/uniformdistn-on-sphere/
	// Note that the coordinate conversion has been optimized for small point clouds (<100k points) by performing it
	// in batch instead of selectively (to make it easy for the compiler to vectorize this operation)
    const T twoPi{ 2. * M_PI };
    const Eigen::Array<T, 1, Eigen::Dynamic> polarAngleOfNormals{ normals.row(2).array().acos() };
    const Eigen::Array<T, 1, Eigen::Dynamic> azimuthAngleOfNormals{ normals.row(1).array().binaryExpr(
        normals.row(0).array(), [&](float y, float x) { return std::fmod(std::atan2(y, x) + twoPi, twoPi); }) };

	///(1) put all points of the data into buckets based on their normal direction
	for (auto pointIndex : pointIndices)
	{
		// Allow for slight approximiation errors
		assert(normals.col(pointIndex).head(3).norm() >= 1.0-0.00001);
		assert(normals.col(pointIndex).head(3).norm() <= 1.0+0.00001);
		// Catch errors where theta will be NaN
		assert((normals(2,pointIndex) <= 1.0) && (normals(2, pointIndex) >= -1.0));

		//Theta = polar angle in [0 ; pi]
		const T theta{polarAngleOfNormals(pointIndex)};
		//Phi = azimuthal angle in [0 ; 2pi]
		const T phi{azimuthAngleOfNormals(pointIndex)};

		// Catch normal space hashing errors
		assert(bucketIdx(theta, phi) < nbBucket);
		idBuckets[bucketIdx(theta, phi)].emplace_back(pointIndex);
	}

	// Remove empty buckets
	idBuckets.erase(std::remove_if(idBuckets.begin(), idBuckets.end(),
				[](const std::vector<int>& bucket) { return bucket.empty(); }),
				idBuckets.end());

	///(2) uniformly pick points from all the buckets until the desired number of points is selected
	for (std::size_t i=0; i<nbSample; i++)
	{
		// Get a random bucket
		std::uniform_int_distribution<std::size_t> uniBucket(0, idBuckets.size()-1);
		const std::size_t curBucketIdx{uniBucket(gen)};
		std::vector<int>& curBucket{idBuckets[curBucketIdx]};

		///(3) A point is randomly picked in a bucket that contains multiple points
		const int idToKeep{curBucket[curBucket.size()-1]};
		curBucket.pop_back();
		keepIndexes.emplace_back(static_cast<std::size_t>(idToKeep));

		// Remove the bucket if it is empty
		if (curBucket.empty()) {
			idBuckets.erase(idBuckets.begin() + curBucketIdx);
		}
	}

	//TODO: evaluate performances between this solution and sorting the indexes
	// We build map of (old index to new index), in case we sample pts at the begining of the pointcloud
	std::vector<std::size_t> indexVector;
	indexVector.resize(nbSample);

	///(4) Sample the point cloud
	std::size_t idx{0};
	for(std::size_t id : keepIndexes)
	{
		//retrieve index from lookup table if sampling in already switched element
		if(id<idx)
			id = indexVector[id];
		//Switch columns id and idx
		cloud.swapCols(idx, id);
		//Maintain new index position
		indexVector[idx] = id;
		//Update index
		++idx;
	}
	cloud.conservativeResize(nbSample);
}

template <typename T>
std::size_t NormalSpaceDataPointsFilter<T>::bucketIdx(T theta, T phi) const
{
	//Theta = polar angle in [0 ; pi] and Phi = azimuthal angle in [0 ; 2pi]
	assert( (theta >= 0.0) && (theta <= static_cast<T>(M_PI)) && "Theta not in [0, Pi]");
	assert( (phi >= 0) && (phi <= 2*static_cast<T>(M_PI)) && "Phi not in [0, 2Pi]");

	// Wrap Theta at Pi
	if (theta == static_cast<T>(M_PI)) { theta = 0.0; };
	// Wrap Phi at 2Pi
	if (phi == 2*static_cast<T>(M_PI)) { phi = 0.0; };
	//                               block number           block size               element number
	return static_cast<std::size_t>( floor(theta/epsilon) * ceil(2.0*M_PI/epsilon) + floor(phi/epsilon) );
}

template struct NormalSpaceDataPointsFilter<float>;
template struct NormalSpaceDataPointsFilter<double>;
