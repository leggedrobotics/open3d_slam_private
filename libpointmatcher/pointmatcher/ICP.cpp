// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include <numeric>
#include <algorithm>
#include "PointMatcher.h"
#include "PointMatcherPrivate.h"
#include "Timer.h"

#include <ceres/ceres.h>

#include "LoggerImpl.h"
#include "TransformationsImpl.h"
#include "DataPointsFiltersImpl.h"
#include "MatchersImpl.h"
#include "OutlierFiltersImpl.h"
#include "ErrorMinimizersImpl.h"
#include "TransformationCheckersImpl.h"
#include "InspectorsImpl.h"
#include "point_cloud_registration.hpp"

// message logger
#include <message_logger/message_logger.hpp>

#ifdef SYSTEM_YAML_CPP
#include "yaml-cpp/yaml.h"
#else
#include "yaml-cpp-pm/yaml.h"
#endif // HAVE_YAML_CPP

using namespace std;
using namespace PointMatcherSupport;

//! Construct an invalid--module-type exception
InvalidModuleType::InvalidModuleType(const std::string& reason) : runtime_error(reason)
{}

//! Protected contstructor, to prevent the creation of this object
template<typename T>
PointMatcher<T>::ICPChainBase::ICPChainBase() :
    prefilteredReadingPtsCount(0), prefilteredReferencePtsCount(0), maxNumIterationsReached(false)
{}

//! virtual desctructor
template<typename T>
PointMatcher<T>::ICPChainBase::~ICPChainBase()
{}

//! Clean chain up, empty all filters and delete associated objects
template<typename T>
void PointMatcher<T>::ICPChainBase::cleanup()
{
    transformations.clear();
    readingDataPointsFilters.clear();
    readingStepDataPointsFilters.clear();
    referenceDataPointsFilters.clear();
    matcher.reset();
    outlierFilters.clear();
    errorMinimizer.reset();
    transformationCheckers.clear();
    inspector.reset();
}

//! Hook to load addition subclass-specific content from the YAML file
template<typename T>
void PointMatcher<T>::ICPChainBase::loadAdditionalYAMLContent(YAML::Node& /*doc*/)
{}

//! Construct an ICP algorithm that works in most of the cases
template<typename T>
void PointMatcher<T>::ICPChainBase::setDefault()
{
    this->cleanup();

    this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::RigidTransformation>());
    //this->readingDataPointsFilters.push_back(std::make_shared<typename DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter>());
    //this->referenceDataPointsFilters.push_back(std::make_shared<typename DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter>());
    //this->outlierFilters.push_back(std::make_shared<typename OutlierFiltersImpl<T>::TrimmedDistOutlierFilter>());
    this->matcher = std::make_shared<typename MatchersImpl<T>::KDTreeMatcher>();
    this->errorMinimizer = std::make_shared<PointToPlaneErrorMinimizer<T>>();
    this->transformationCheckers.push_back(std::make_shared<typename TransformationCheckersImpl<T>::CounterTransformationChecker>());
    this->transformationCheckers.push_back(std::make_shared<typename TransformationCheckersImpl<T>::DifferentialTransformationChecker>());
    this->inspector = std::make_shared<typename InspectorsImpl<T>::NullInspector>();
}

//! Construct an ICP algorithm from a YAML file
template<typename T>
void PointMatcher<T>::ICPChainBase::loadFromYaml(std::istream& in)
{
    this->cleanup();

    YAML::Parser parser(in);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    typedef set<string> StringSet;
    StringSet usedModuleTypes;

    // Fix for issue #6: compilation on gcc 4.4.4
    //PointMatcher<T> pm;
    const PointMatcher& pm = PointMatcher::get();

    {
        // NOTE: The logger needs to be initialize first to allow ouput from other contructors
        boost::mutex::scoped_lock lock(loggerMutex);
        usedModuleTypes.insert(createModuleFromRegistrar("logger", doc, pm.REG(Logger), logger));
    }
    usedModuleTypes.insert(createModulesFromRegistrar("readingDataPointsFilters", doc, pm.REG(DataPointsFilter), readingDataPointsFilters));
    usedModuleTypes.insert(
        createModulesFromRegistrar("readingStepDataPointsFilters", doc, pm.REG(DataPointsFilter), readingStepDataPointsFilters));
    usedModuleTypes.insert(
        createModulesFromRegistrar("referenceDataPointsFilters", doc, pm.REG(DataPointsFilter), referenceDataPointsFilters));
    //usedModuleTypes.insert(createModulesFromRegistrar("transformations", doc, pm.REG(Transformation), transformations));
    usedModuleTypes.insert(createModuleFromRegistrar("matcher", doc, pm.REG(Matcher), matcher));
    usedModuleTypes.insert(createModulesFromRegistrar("outlierFilters", doc, pm.REG(OutlierFilter), outlierFilters));
    usedModuleTypes.insert(createModuleFromRegistrar("errorMinimizer", doc, pm.REG(ErrorMinimizer), errorMinimizer));

    // Read the cost function type.
    this->costFunctionNameStr_ = nodeVal("errorMinimizer", doc);

    // Read localizability awareness parameters.
    if (readLocalizabilityAwarenessParameters("degeneracyAwareness", doc))
    {
        usedModuleTypes.insert("degeneracyAwareness");
    }

    // Read localizability debug parameters.
    if (readLocalizabilityDebug("degeneracyDebug", doc))
    {
        usedModuleTypes.insert("degeneracyDebug");
    }
    else
    {
        std::cout << "No degeneracy debug parameters found." << std::endl;
        usedModuleTypes.insert("degeneracyDebug");
    }

    // Read localizability print parameters.
    if (readLocalizabilityPrint("printingDegeneracy", doc))
    {
        usedModuleTypes.insert("printingDegeneracy");
    }
    else
    {
        std::cout << "No degeneracy printing parameters are found" << std::endl;
        usedModuleTypes.insert("degeneracyDebug");
    }

    // Read localizability debug parameters.
    if (readCeresDegeneracyAnalysis("ceresDegeneracyAnalysis", doc))
    {
        usedModuleTypes.insert("ceresDegeneracyAnalysis");
    }
    else
    {
        std::cout << "TEST TEST No ceresDegeneracyAnalysis parameters found. TEST  TEST" << std::endl;
        usedModuleTypes.insert("ceresDegeneracyAnalysis");
    }

    // See if to use a rigid transformation
    if (this->costFunctionNameStr_ != "PointToPointSimilarityErrorMinimizer")
        this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::RigidTransformation>());
    else
        this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::SimilarityTransformation>());

    usedModuleTypes.insert(
        createModulesFromRegistrar("transformationCheckers", doc, pm.REG(TransformationChecker), transformationCheckers));
    usedModuleTypes.insert(createModuleFromRegistrar("inspector", doc, pm.REG(Inspector), inspector));


    // FIXME: this line cause segfault when there is an error in the yaml file...
    //loadAdditionalYAMLContent(doc);

    // check YAML entries that do not correspend to any module
    for (YAML::Iterator moduleTypeIt = doc.begin(); moduleTypeIt != doc.end(); ++moduleTypeIt)
    {
        string moduleType;
        moduleTypeIt.first() >> moduleType;
        if (usedModuleTypes.find(moduleType) == usedModuleTypes.end())
            throw InvalidModuleType((boost::format("Module type %1% does not exist") % moduleType).str());
    }
}

//! Return the remaining number of points in reading after prefiltering but before the iterative process
template<typename T>
unsigned PointMatcher<T>::ICPChainBase::getPrefilteredReadingPtsCount() const
{
    return prefilteredReadingPtsCount;
}

//! Return the remaining number of points in the reference after prefiltering but before the iterative process
template<typename T>
unsigned PointMatcher<T>::ICPChainBase::getPrefilteredReferencePtsCount() const
{
    return prefilteredReferencePtsCount;
}

//! Return the flag that informs if we reached the maximum number of iterations during the last iterative process
template<typename T>
bool PointMatcher<T>::ICPChainBase::getMaxNumIterationsReached() const
{
    return maxNumIterationsReached;
}

//! Instantiate modules if their names are in the YAML file
template<typename T>
template<typename R>
const std::string& PointMatcher<T>::ICPChainBase::createModulesFromRegistrar(const std::string& regName, const YAML::Node& doc,
                                                                             const R& registrar,
                                                                             std::vector<std::shared_ptr<typename R::TargetType>>& modules)
{
    const YAML::Node* reg = doc.FindValue(regName);
    if (reg)
    {
        //cout << regName << endl;
        for (YAML::Iterator moduleIt = reg->begin(); moduleIt != reg->end(); ++moduleIt)
        {
            const YAML::Node& module(*moduleIt);
            modules.push_back(registrar.createFromYAML(module));
        }
    }
    return regName;
}

//! Instantiate a module if its name is in the YAML file
template<typename T>
template<typename R>
const std::string& PointMatcher<T>::ICPChainBase::createModuleFromRegistrar(const std::string& regName, const YAML::Node& doc,
                                                                            const R& registrar,
                                                                            std::shared_ptr<typename R::TargetType>& module)
{
    const YAML::Node* reg = doc.FindValue(regName);
    if (reg)
    {
        //cout << regName << endl;
        module = registrar.createFromYAML(*reg);
    }
    else
        module.reset();
    return regName;
}

template<typename T>
std::string PointMatcher<T>::ICPChainBase::nodeVal(const std::string& regName, const PointMatcherSupport::YAML::Node& doc)
{
    const YAML::Node* reg = doc.FindValue(regName);
    if (reg)
    {
        std::string name;
        Parametrizable::Parameters params;
        PointMatcherSupport::getNameParamsFromYAML(*reg, name, params);
        return name;
    }
    return "";
}

template<typename T>
bool PointMatcher<T>::ICPChainBase::readLocalizabilityDebug(const std::string& yamlKey, const PointMatcherSupport::YAML::Node& doc)
{
    const YAML::Node* reg = doc.FindValue(yamlKey);
    if (reg == nullptr)
    {
        MELO_WARN("LOCALIZABILITY DEBUG NOT SET");
        return false;
    }

    std::string methodName{ "" };
    Parametrizable::Parameters params;
    PointMatcherSupport::getNameParamsFromYAML(*reg, methodName, params);

    if (methodName == "Enabled")
    {
        localizabilityDetectionParameters.isDebugModeENabled_ = true;
    }
    else
    {
        // If not set correctly set none and return false.
        MELO_ERROR("Debug mode is not set or False: '%s'", methodName.c_str());
        localizabilityDetectionParameters.isDebugModeENabled_ = false;
        return false;
    }

    MELO_INFO("Degeneracy awareness method: '%s'", methodName.c_str());

    return true;
}

template<typename T>
bool PointMatcher<T>::ICPChainBase::readLocalizabilityPrint(const std::string& yamlKey, const PointMatcherSupport::YAML::Node& doc)
{
    const YAML::Node* reg = doc.FindValue(yamlKey);
    if (reg == nullptr)
    {
        MELO_WARN("LOCALIZABILITY Printing NOT SET");
        return false;
    }

    std::string methodName{ "" };
    Parametrizable::Parameters params;
    PointMatcherSupport::getNameParamsFromYAML(*reg, methodName, params);
    localizabilityDetectionParameters.isPrintingEnabled_ = true;
    /*if (methodName == "Enabled")
    {
        localizabilityDetectionParameters.isPrintingEnabled_ = true;
    }
    else
    {
        // If not set correctly set none and return false.
        MELO_ERROR("Printing is not set or False: '%s'", methodName.c_str());
        localizabilityDetectionParameters.isPrintingEnabled_ = false;
        return false;
    }*/

    MELO_INFO("Degeneracy awareness method: '%s'", methodName.c_str());

    return true;
}

template<typename T>
bool PointMatcher<T>::ICPChainBase::readCeresDegeneracyAnalysis(const std::string& yamlKey, const PointMatcherSupport::YAML::Node& doc)
{
    const YAML::Node* reg = doc.FindValue(yamlKey);
    if (reg == nullptr)
    {
        MELO_WARN("readCeresDegeneracyAnalysis NOT SET");
        return false;
    }

    std::string methodName{ "" };
    Parametrizable::Parameters params;
    PointMatcherSupport::getNameParamsFromYAML(*reg, methodName, params);

    if (methodName == "CeresDegeneracyState")
    {
        if ((params.count("isActive") > 0))
        {
            if (PointMatcherSupport::lexical_cast<T>(params.at("isActive")) == T(0))
            {
                degeneracySolverOptions_.isEnabled_ = false;
                MELO_WARN("================ Ceres bases solvers are DISABLED ================");
                return true;
            }
            else
            {
                MELO_WARN("================ Ceres bases solvers are ENABLED ================");
                degeneracySolverOptions_.isEnabled_ = true;
            }
        }
        else{
            return false;
        }

        if ((params.count("useSophusParametrization") > 0))
        {
            if (PointMatcherSupport::lexical_cast<T>(params.at("useSophusParametrization")) > T(0))
            {
                MELO_WARN("================ Sophus Parametrization is Enabled ================");
                degeneracySolverOptions_.useSophusParametrization_ = true;
            }
            else
            {
                MELO_WARN("================ Sophus Parametrization is Disabled ================");
                degeneracySolverOptions_.useSophusParametrization_ = false;
            }
        }
        else
        {
            return false;
        }

        if ((params.count("useAngleAxisParametrization") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useAngleAxisParametrization")) > T(0))
            {
                MELO_WARN("================ AngleAxis Parametrization is Enabled ================");
                degeneracySolverOptions_.useAngleAxisParametrization_ = true;
            }
            else
            {
                MELO_WARN("================ AngleAxis Parametrization is Disabled ================");
                degeneracySolverOptions_.useAngleAxisParametrization_ = false;
            }
        }
        else
        {
            return false;
        }

        if ((params.count("useSophusAutoDiffParametrization") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useSophusAutoDiffParametrization")) > T(0))
            {
                MELO_WARN("================ Sophus Autodiff Jacobian is Enabled ================");
                degeneracySolverOptions_.useSophusAutoDiffParametrization_ = true;
            }
            else
            {
                MELO_WARN("================ Sophus Autodiff Jacobian is Disabled ================");
                degeneracySolverOptions_.useSophusAutoDiffParametrization_ = false;
            }
        }else
        {
            return false;
        }
        
        if ((params.count("usePointToPointCost") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("usePointToPointCost")) > T(0))
            {
                MELO_WARN("================ Point to Point Cost is Enabled ================");
                degeneracySolverOptions_.usePointToPoint_ = true;
            }
            else
            {
                MELO_WARN("================ Point to Point Cost is Disabled ================");
                degeneracySolverOptions_.usePointToPoint_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("usePointToPlaneCost") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("usePointToPlaneCost")) > T(0))
            {
                MELO_WARN("================ Point to Plane Cost is Enabled ================");
                degeneracySolverOptions_.usePointToPlane_ = true;
            }
            else
            {
                MELO_WARN("================ Point to Plane Cost is DISABLED ================");
                degeneracySolverOptions_.usePointToPlane_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("usePointToLineCost") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("usePointToLineCost")) > T(0))
            {
                MELO_WARN("================ Point to Line Cost is Enabled ================");
                degeneracySolverOptions_.usePointToLine_ = true;
            }
            else
            {
                MELO_WARN("================ Point to Line Cost is DISABLED ================");
                degeneracySolverOptions_.usePointToLine_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("useSymmetricPointToPlaneCost") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useSymmetricPointToPlaneCost")) > T(0))
            {
                MELO_WARN("================ Symmetric Point to Plane Cost is Enabled ================");
                degeneracySolverOptions_.useSymmetricPointToPlane_ = true;
            }
            else
            {
                MELO_WARN("================ Symmetric Point to Plane Cost is DISABLED ================");
                degeneracySolverOptions_.useSymmetricPointToPlane_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("useBoundConstraints") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useBoundConstraints")) > T(0))
            {
                MELO_WARN("================ Bound constraints are Enabled ================");
                degeneracySolverOptions_.useBoundConstraints_ = true;
            }
            else
            {
                MELO_WARN("================ Bound constraints are DISABLED ================");
                degeneracySolverOptions_.useBoundConstraints_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("useSixDofRegularization") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useSixDofRegularization")) == 1.0)
            {
                MELO_WARN("================ Six DOF Regularization is Enabled ================");
                degeneracySolverOptions_.useSixDofRegularization_ = true;
            }
            else
            {
               MELO_WARN("================ Six DOF Regularization is DISABLED ================");
                degeneracySolverOptions_.useSixDofRegularization_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("useThreeDofRegularization") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("useThreeDofRegularization")) == 1.0)
            {
                MELO_WARN("================ Three DOF Regularization is Enabled ================");
                degeneracySolverOptions_.useThreeDofRegularization_ = true;
            }
            else
            {
               MELO_WARN("================ Three DOF Regularization is DISABLED ================");
                degeneracySolverOptions_.useThreeDofRegularization_ = false;
            }
        }else
        {
            return false;
        }

        if ((params.count("regularizationWeight") > 0))
        {

            if (PointMatcherSupport::lexical_cast<T>(params.at("regularizationWeight")) > T(0))
            {
                MELO_WARN("================ Regularization Weight is SET ================");
                degeneracySolverOptions_.regularizationWeight_ = PointMatcherSupport::lexical_cast<float>(params.at("regularizationWeight"));
            }else{
                return false;
            }
        }else
        {
            return false;
        }

    }else{
        return false;
    }

    return true;
}


template<typename T>
bool PointMatcher<T>::ICPChainBase::readLocalizabilityAwarenessParameters(const std::string& yamlKey,
                                                                          const PointMatcherSupport::YAML::Node& doc)
{
    const YAML::Node* reg = doc.FindValue(yamlKey);
    if (reg == nullptr)
    {
        MELO_WARN("Degeneracy awareness configuration not present in ICP parameters.");
        return false;
    }

    std::string methodName{ "" };
    Parametrizable::Parameters params;
    PointMatcherSupport::getNameParamsFromYAML(*reg, methodName, params);

    if (methodName == "None")
    {
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kNone;
    }
    else if (methodName == "SolutionRemapping")
    {
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kSolutionRemapping;
        if (params.count("threshold") > 0)
        {
            localizabilityDetectionParameters.solutionRemappingThreshold = PointMatcherSupport::lexical_cast<T>(params.at("threshold"));
        }
        else
        {
            return false;
        }
        if (params.count("use2019") > 0)
        {
            if (PointMatcherSupport::lexical_cast<T>(params.at("use2019")) == 1.0)
            {
                MELO_WARN("Sol Remap 2019 set to TRUE");
                localizabilityDetectionParameters.useSolutionRemapping2019 = true;
            }
            else
            {
                MELO_WARN("Sol Remap 2019 set to false");
                localizabilityDetectionParameters.useSolutionRemapping2019 = false;
            }
        }
        else
        {
            return false;
        }
    }
    else if (methodName == "OptimizedEqualityConstraints")
    {
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kOptimizedEqualityConstraints;
        if (params.count("enoughInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.enoughInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("enoughInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("insufficientInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.insufficientInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("insufficientInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalMinimalAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalMinimalAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalMinimalAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalStrongAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalStrongAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalStrongAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
    }
    else if (methodName == "EqualityConstraints")
    {
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kEqualityConstraints;
        if (params.count("highInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.highInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("highInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("enoughInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.enoughInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("enoughInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("insufficientInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.insufficientInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("insufficientInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalMinimalAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalMinimalAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalMinimalAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalStrongAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalStrongAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalStrongAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
    }
    else if (methodName == "InequalityConstraints")
    {
        std::cout << "INEQUALITY CONSTRAINTS ARE ENABLED ---------- " << std::endl;
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kInequalityConstraints;
        if (params.count("highInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.highInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("highInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("enoughInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.enoughInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("enoughInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("insufficientInformationThreshold") > 0)
        {
            localizabilityDetectionParameters.insufficientInformationThreshold =
                PointMatcherSupport::lexical_cast<T>(params.at("insufficientInformationThreshold"));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalMinimalAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalMinimalAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalMinimalAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
        if (params.count("point2NormalStrongAlignmentAngleThreshold") > 0)
        {
            localizabilityDetectionParameters.point2NormalStrongAlignmentCosineThreshold =
                std::cos(PointMatcherSupport::convertAngleInDegreesToRadians(
                    PointMatcherSupport::lexical_cast<T>(params.at("point2NormalStrongAlignmentAngleThreshold"))));
        }
        else
        {
            return false;
        }
    }
    else
    {
        // If not set correctly set none and return false.
        MELO_ERROR("Unrecognized degeneracy awareness method: '%s'", methodName.c_str());
        localizabilityDetectionParameters.degeneracyAwarenessMethod = DegeneracyAwarenessMethod::kNone;
        return false;
    }

    MELO_INFO("Degeneracy awareness method: '%s'", methodName.c_str());

    return true;
}

template struct PointMatcher<float>::ICPChainBase;
template struct PointMatcher<double>::ICPChainBase;


//! Perform ICP and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator()(const DataPoints& readingIn,
                                                                                    const DataPoints& referenceIn)
{
    const int dim = readingIn.features.rows();
    const TransformationParameters identity = TransformationParameters::Identity(dim, dim);
    return this->compute(readingIn, referenceIn, identity);
}

//! Perform ICP from initial guess and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator()(
    const DataPoints& readingIn,
    const DataPoints& referenceIn,
    const TransformationParameters& initialTransformationParameters)
{
    return this->compute(readingIn, referenceIn, initialTransformationParameters);
}

//! Perform ICP from initial guess and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::compute(const DataPoints& readingIn,
                                                                                 const DataPoints& referenceIn,
                                                                                 const TransformationParameters& T_refIn_dataIn,
                                                                                 const bool initializeMatcherWithInputReference)
{
    // Ensuring minimum definition of components
    if (!this->matcher)
        throw runtime_error("You must setup a matcher before running ICP");
    if (!this->errorMinimizer)
        throw runtime_error("You must setup an error minimizer before running ICP");
    if (!this->inspector)
        throw runtime_error("You must setup an inspector before running ICP");

    this->inspector->init();

    timer t; // Print how long take the algo

    // Initialize the matcher and reference holder only if required explicitly.
    if (initializeMatcherWithInputReference || !this->matcherIsInitialized)
    {
        initReference(referenceIn);
    }

    // statistics on last step
    this->inspector->addStat("ReferencePreprocessingDuration", t.elapsed());
    this->inspector->addStat("ReferenceInPointCount", referenceIn.features.cols());
    this->inspector->addStat("ReferencePointCount", this->referenceFiltered.features.cols());
    LOG_INFO_STREAM("PointMatcher::icp - reference pre-processing took " << t.elapsed() << " [s]");
    this->prefilteredReferencePtsCount = this->referenceFiltered.features.cols();

    return computeWithTransformedReference(readingIn, this->referenceFiltered, this->T_refIn_refMean, T_refIn_dataIn);
}
template<typename T>
bool PointMatcher<T>::ICP::initReference(const DataPoints& referenceIn)
{
    const long int nbPtsReference{ referenceIn.features.cols() };
    if (nbPtsReference == 0)
    {
        this->matcherIsInitialized = false;
        return false;
    }

    // Reset reference and transformation.
    const long int dim{ referenceIn.features.rows() };
    this->referenceFiltered = referenceIn;

    // Reset transformation from reference frame to reference centroid.
    this->T_refIn_refMean = Matrix::Identity(dim, dim);

    // Apply filters to reference.
    this->referenceDataPointsFilters.init();
    this->referenceDataPointsFilters.apply(this->referenceFiltered);

    // Create intermediate frame at the center of mass of reference pts cloud
    //  this helps to solve for rotations
    const long int nbPtsReferenceFiltered{ this->referenceFiltered.features.cols() };
    const Vector meanReference{ this->referenceFiltered.features.rowwise().sum() / nbPtsReferenceFiltered };
    this->T_refIn_refMean.block(0, dim - 1, dim - 1, 1) = meanReference.head(dim - 1);

    // Readjust reference position:
    // 	Reference was originally expressed in the frame <refIn>
    // 	from here on reference is expressed in frame <refMean>
    // 	Shortcut to do T_refIn_refMean.inverse() * reference
    this->referenceFiltered.features.topRows(dim - 1).colwise() -= meanReference.head(dim - 1);

    // Init matcher with reference points centered. on its mean
    this->matcher->resetVisitCount();
    this->matcher->init(this->referenceFiltered);
    this->matcherIsInitialized = true;

    return true;
}

//! Perferm ICP using an already-transformed reference and with an already-initialized matcher
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::computeWithTransformedReference(
    const DataPoints& readingIn,
    const DataPoints& reference,
    const TransformationParameters& T_refIn_refMean,
    const TransformationParameters& T_refIn_dataIn)
{
    const int dim(reference.features.rows());

    if (T_refIn_dataIn.cols() != T_refIn_dataIn.rows())
    {
        throw runtime_error("The initial transformation matrix must be squared.");
    }
    if (dim != T_refIn_dataIn.cols())
    {
        throw runtime_error("The shape of initial transformation matrix must be NxN. "
                            "Where N is the number of rows in the read/reference scans.");
    }

    // Get reference to the parameters to be passed to the solver.
    LocalizabilityParametersForErrorMinimization localizabilityParametersForErrorMinimization;
    localizabilityParametersForErrorMinimization.degeneracyAwarenessMethod =
        this->localizabilityDetectionParameters.degeneracyAwarenessMethod;

    // Print how long take the algo
    timer t;
    std::shared_ptr<PointMatcher<T>::DataPointsFilter> boundingBoxFilter;
    bool keepInside = true;
    /*
    boundingBoxFilter = PointMatcher<T>::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter",
                                                                        { { "xMin", PointMatcherSupport::toParam(-10.0f) },
                                                                          { "xMax", PointMatcherSupport::toParam(10.0f) },
                                                                          { "yMin", PointMatcherSupport::toParam(-10.0f) },
                                                                          { "yMax", PointMatcherSupport::toParam(10.0f) },
                                                                          { "zMin", PointMatcherSupport::toParam(-0.5f) },
                                                                          { "zMax", PointMatcherSupport::toParam(20.0f) },
                                                                          { "removeInside", PointMatcherSupport::toParam(!keepInside) } });
    */
    boundingBoxFilter =
        PointMatcher<T>::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter",
                                                                { { "xMin", PointMatcherSupport::toParam(-25.0f) },
                                                                  { "xMax", PointMatcherSupport::toParam(25.0f) },
                                                                  { "yMin", PointMatcherSupport::toParam(-25.0f) },
                                                                  { "yMax", PointMatcherSupport::toParam(25.0f) },
                                                                  { "zMin", PointMatcherSupport::toParam(-0.5f) },
                                                                  { "zMax", PointMatcherSupport::toParam(40.0f) },
                                                                  { "removeInside", PointMatcherSupport::toParam(!keepInside) } });


    // Apply readings filters
    // reading is express in frame <dataIn>
    DataPoints reading(readingIn);
    this->readingDataPointsFilters.init();
    this->readingDataPointsFilters.apply(reading);

    //boundingBoxFilter->inPlaceFilter(reading);

    readingFiltered = reading;

    // Reajust reading position: f
    // from here reading is express in frame <refMean>
    TransformationParameters T_refMean_dataIn = T_refIn_refMean.inverse() * T_refIn_dataIn;

    //if(this->transformations.front()->checkParameters(T_refMean_dataIn) == false)
    //{
        //T_refMean_dataIn = this->transformations.front()->correctParameters(T_refMean_dataIn);
    //}


    this->transformations.apply(reading, T_refMean_dataIn);

    // Prepare reading filters used in the loop
    this->readingStepDataPointsFilters.init();

    // Since reading and reference are express in <refMean>
    // the frame <refMean> is equivalent to the frame <iter(0)>
    TransformationParameters T_iter = Matrix::Identity(dim, dim);

    bool iterate(true);
    this->maxNumIterationsReached = false;
    this->transformationCheckers.init(T_iter, iterate);

    size_t iterationCount(0);

    // statistics on last step
    this->inspector->addStat("ReadingPreprocessingDuration", t.elapsed());
    this->inspector->addStat("ReadingInPointCount", readingIn.features.cols());
    this->inspector->addStat("ReadingPointCount", reading.features.cols());
    LOG_INFO_STREAM("PointMatcher::icp - reading pre-processing took " << t.elapsed() << " [s]");
    this->prefilteredReadingPtsCount = reading.features.cols();
    t.restart();

    std::vector<T> scalabilityVect;
    std::vector<T> NbMatchesVet;

    //TransformationParameters T_convert_from_base_to_lidar_mat = Matrix::Identity(4,4);
    Eigen::Transform<T, 3, Eigen::Affine> T_convert_from_base_to_lidar;
    Vector unitZ(3, 1);
    unitZ << 0, 0, 1;
    T_convert_from_base_to_lidar = Eigen::AngleAxis<T>(M_PI, unitZ);
    Vector transMe(3, 1);
    transMe << -0.310, 0.000, -0.159;
    T_convert_from_base_to_lidar.translation() = transMe;
    localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat = T_convert_from_base_to_lidar.matrix();

    //Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();

    // iterations
    while (iterate)
    {
        // Check whether it is the first iteration.
        localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_ = (iterationCount == 0u);

        //-----------------------------
        // Set the reading cloud
        DataPoints stepReading(reading);

        //-----------------------------
        // Apply step filter
        this->readingStepDataPointsFilters.apply(stepReading);

        //if(this->transformations.front()->checkParameters(T_iter) == false)
        //{
            //T_iter = this->transformations.front()->correctParameters(T_iter);
        //}

        //-----------------------------
        // Transform Readings
        this->transformations.apply(stepReading, T_iter);

        //-----------------------------
        // Match to closest point in Reference
        this->matches = this->matcher->findClosests(stepReading);

        //-----------------------------
        // Detect outliers
        this->outlierWeights = this->outlierFilters.compute(stepReading, reference, this->matches);

        assert(this->outlierWeights.rows() == this->matches.ids.rows());
        assert(this->outlierWeights.cols() == this->matches.ids.cols());

        // Get the matches
        ErrorElements matchedPoints(stepReading, reference, this->outlierWeights, this->matches);
        ErrorElements matchedPointsCeres(stepReading, reference, this->outlierWeights, this->matches);
        MELO_DEBUG_STREAM("Number of matches : " << matchedPoints.reading.getNbPoints() << " At iteration: " << iterationCount);

        this->nbMatches_ = matchedPoints.reading.getNbPoints();
        this->plottingDistributionContribution_.conservativeResize(matchedPoints.reading.getNbPoints(), Eigen::NoChange);
        this->localizabilityDetectionParameters.transformationToOptimizationFrame = T_refMean_dataIn;
        this->localizabilityDetectionParameters.numberOfPoints = matchedPoints.reading.getNbPoints();

        // Calculate optimization hessian.
        calculateOptimizationHessian(localizabilityParametersForErrorMinimization.A_,
                                     localizabilityParametersForErrorMinimization.b_,
                                     matchedPoints,
                                     localizabilityParametersForErrorMinimization);

        timer scalabilityStudy; // Print how long take the algo
        scalabilityStudy.restart();

        switch (this->localizabilityDetectionParameters.degeneracyAwarenessMethod)
        {
            case DegeneracyAwarenessMethod::kNone: {
                // Not required to do anything. Skip all the localizability detection.
                if (localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
                {
                    MELO_DEBUG_STREAM("ICP Localizability Method: None");
                }
            }
            break;
            case DegeneracyAwarenessMethod::kSolutionRemapping: {
                if (this->costFunctionNameStr_ == "PointToPlaneErrorMinimizer")
                {
                    if (localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
                    {
                        MELO_DEBUG_STREAM("ICP Localizability Method: Solution Remapping");
                    }

                    if (!detectLocalizabilityWithSolutionRemappingMethod(
                            localizabilityParametersForErrorMinimization,
                            this->localizabilityDetectionParameters.solutionRemappingThreshold))
                    {
                        localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_ = true;
                    }
                }
                else
                {
                    MELO_WARN_THROTTLE_STREAM(10,
                                              "Cost function is "
                                                  << this->costFunctionNameStr_
                                                  << ". Robust registration methods expect point-to-plane. (Throttled 10s)");
                }
            }
            break;
            case DegeneracyAwarenessMethod::kOptimizedEqualityConstraints: {
                if (this->costFunctionNameStr_ == "PointToPlaneErrorMinimizer")
                {

                    if (!detectLocalizabilityWithOptimizedMethod(matchedPoints, localizabilityParametersForErrorMinimization))
                    {
                        MELO_WARN("Optimized equality constrained ICP failed. Will return the prior.");
                        localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_ = true;
                    }

                    if (localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
                    {
                        MELO_DEBUG_STREAM("ICP Localizability Method: Optimized.");
                        MELO_DEBUG_STREAM(
                            "Translation Localizability : "
                            << localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.transpose());
                        MELO_DEBUG_STREAM(
                            "Rotation Localizability : "
                            << localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.transpose());
                    }
                }
                else
                {
                    MELO_WARN_THROTTLE_STREAM(10,
                                              "Cost function is "
                                                  << this->costFunctionNameStr_
                                                  << ". Robust registration methods expect point-to-plane. (Throttled 10s)");
                }
            }
            break;
            case DegeneracyAwarenessMethod::kEqualityConstraints:
            case DegeneracyAwarenessMethod::kInequalityConstraints: {
                if (this->costFunctionNameStr_ == "PointToPlaneErrorMinimizer")
                {

                    if (detectLocalizabilityWithTernaryLevelDetection(matchedPoints, localizabilityParametersForErrorMinimization))
                    {
                        MELO_DEBUG_THROTTLE_STREAM(10, "Successful localizability detection. (Throttled 10s)");
                    }
                    else
                    {
                        MELO_DEBUG_THROTTLE_STREAM(10, "Early return on localizability detection. (Throttled 10s)");
                    }
                    /*

                    //if (localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
                    //{
                        MELO_DEBUG_STREAM("ICP Localizability Method: Advanced_inequality or Advanced.");
                        MELO_INFO_STREAM(
                            "Translation Localizability : "
                            << localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.transpose());
                        MELO_INFO_STREAM(
                            "Rotation Localizability : "
                            << localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.transpose());
                    //}

                        Eigen::Matrix<T, 3, 6> toPharosArrows = Matrix::Zero(3, 6);

                        for (size_t j = 0; j < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.rows(); j++){
                            if(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(j).value() == 0){
                                toPharosArrows.col(j) = localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(j);
                            }
                        }

                        for (size_t j = 0; j < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.rows(); j++){
                            if(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(j).value() == 0){
                                toPharosArrows.col(j+3) = localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(j);

                            }
                        }

                        this->toPharosArrows = toPharosArrows;

                        */
                }
                else
                {
                    MELO_WARN_THROTTLE_STREAM(10,
                                              "Cost function is "
                                                  << this->costFunctionNameStr_
                                                  << ". Robust registration methods expect point-to-plane. (Throttled 10s)");
                }
            }
            break;
            default:
                // Not required to do anything. Skip all the localizability detection.
                if (localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
                {
                    MELO_DEBUG_STREAM("ICP Localizability Method: None");
                }
        }

        // Based on localizabiltiy detection success break the ICP loop.
        if (localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_)
        {
            MELO_ERROR_STREAM("RETURNING PRIOR WITH BREAK");
            break;
        }

        //-----------------------------
        // Dump
        //this->inspector->dumpIteration(
        //	iterationCount, T_iter, reference, stepReading, this->matches, this->outlierWeights, this->transformationCheckers
        //);

        //-----------------------------
        // Error minimization
        // equivalent to:
        //   T_iter(i+1)_iter(0) = T_iter(i+1)_iter(i) * T_iter(i)_iter(0)
        //T_iter = this->errorMinimizer->compute(
        //	stepReading, reference, this->outlierWeights, this->matches, localizabilityParametersForErrorMinimization) * T_iter;

        // Register degeneracy information.
        this->degenerateDirectionsInMapFrame_ = registerDegenerateDirections(localizabilityParametersForErrorMinimization);

        if(this->degeneracySolverOptions_.useThreeDofRegularization_){
            for (Eigen::Index i = 0; i < this->degenerateDirectionsInMapFrame_.cols(); ++i)
            {
                this->ceresEigVect_.col(i).topRows(3) = this->degenerateDirectionsInMapFrame_.col(i);
            }
        }
        //std::cout << "Registered categories " << std::endl;
        //std::cout << "degenerateDirectionsInMapFrame_: " << this->degenerateDirectionsInMapFrame_ << std::endl;

        ceres::Solver::Options options;

        //options.preconditioner_type = ceres::SCHUR_JACOBI;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
        //options.use_explicit_schur_complement=true;

        options.linear_solver_type = ceres::DENSE_QR; // DENSE_SVD DENSE_QR
        //options.trust_region_strategy_type = ceres::DOGLEG;
        //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        //options.use_nonmonotonic_steps = true;  // true
        //options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 30; // 100
        //options.function_tolerance = 1.0e-16;  // 1.0e-16;
        options.num_threads = 1;
        //options->max_solver_time_in_seconds = 12 * 60 * 60; 
        //options.minimizer_type = ceres::LINE_SEARCH;
        //options.gradient_tolerance = 1e-50;
        //options.function_tolerance = 1e-50;
        //options.parameter_tolerance = 1e-50;

        //options.gradient_check_relative_precision = 1e-3;
        //options.check_gradients = true;



        // dont forget this here
        TransformationParameters real =
            this->errorMinimizer->computeFromErrorElements(matchedPoints, localizabilityParametersForErrorMinimization);

        if (this->degeneracySolverOptions_.isEnabled_)
        {
            //MELO_INFO_STREAM(message_logger::color::white << "CERES OPTIMIZATION IS ENABLED: ");
            ceres::Solver::Summary summary;
            
            Eigen::Matrix<float, -1, -1> source_normals = matchedPointsCeres.reading.getDescriptorViewByName("normals").template cast<float>();
            Eigen::Matrix<float, -1, -1> source_points = matchedPointsCeres.reading.features.template cast<float>();

            Eigen::Matrix<float, -1, -1> reference_normals =
                matchedPointsCeres.reference.getDescriptorViewByName("normals").template cast<float>();
            Eigen::Matrix<float, -1, -1> reference_points = matchedPointsCeres.reference.features.template cast<float>();

            //Eigen::Matrix<float, -1, -1> reference_normals = matchedPoints.reading.getDescriptorViewByName("matched_reference_normals").template cast<float>();
            PointCloudRegistrationCeres registration = PointCloudRegistrationCeres(
                source_points, reference_points, source_normals, reference_normals, this->ceresEigVect_.template cast<float>(), this->degeneracySolverOptions_);

            registration.solve(options, &summary, this->degeneracySolverOptions_, localizabilityParametersForErrorMinimization.constraintMappingMatrix_);
            //std::cout << summary.FullReport();

            //Eigen::Transform<double, 3, Eigen::Affine> resultingTransform = registration.transformationSophus();
            //Eigen::Transform<double, 3, Eigen::Affine> resultingTransform = registration.transformation();
            Eigen::Transform<double, 3, Eigen::Affine> resultingTransform;
            
            if(this->degeneracySolverOptions_.useSymmetricPointToPlane_){
                resultingTransform = registration.transformation();
            }else{
                resultingTransform = registration.transformationSeparate();
            }

            //resultingTransform= registration.transformation();
            //resultingTransform= registration.transformationSophus();

            //resultingTransform = Eigen::AngleAxis<double>(state.head(3).norm(), state.head(3).normalized());
            //resultingTransform.translation() = state.segment(3, 3);
            //std::cout << "Resulting Transform: " << std::endl;
            //std::cout << resultingTransform.matrix() << std::endl;
            //Eigen::Matrix<float, -1, -1> res = resultingTransform.matrix().template cast<float>();
            //TransformationParameters tesst = resultingTransform.matrix().template cast<T>();
            T_iter = resultingTransform.matrix().template cast<T>() * T_iter;
        }else{
            T_iter = real * T_iter;
        }

        // Reset the detection vector to not leak data.
        this->ceresEigVect_ = Matrix::Zero(6, 6);

        /*
        MELO_INFO_STREAM("Ceres transformation vector 0: " << state.row(0));
        MELO_INFO_STREAM("Ceres transformation vector 1: " << state.row(1));
        MELO_INFO_STREAM("Ceres transformation vector 2: " << state.row(2));

        MELO_INFO_STREAM("Ceres transformation vector 3: " << state.row(3));
        MELO_INFO_STREAM("Ceres transformation vector 4: " << state.row(4));
        MELO_INFO_STREAM("Ceres transformation vector 5: " << state.row(5));
        */
        //MELO_INFO_STREAM("PointMatcher::icp - " << iterationCount << " iterations took " << t.elapsed() << " [s]");


        //T_iter = real * T_iter;

        if (this->localizabilityDetectionParameters.isDebugModeENabled_)
        {
            this->errorMinimizer->appendIterationResidualError(
                this->errorMinimizer->getResidualError(stepReading, reference, this->outlierWeights, this->matches));
            this->errorMinimizer->appendTransformation(T_refIn_refMean * T_iter * T_refMean_dataIn);
            this->errorMinimizer->appendErrorElements(this->errorMinimizer->getErrorElements());
        }

        // Old version
        //T_iter = T_iter * this->errorMinimizer->compute(
        //	stepReading, reference, outlierWeights, matches);

        // To Pharos categorization. (Rotation)
        for (size_t i = 0; i < 3; i++)
        {
            if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(i).value()
                == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
            {
                if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                        .rotationConstraintValues_.row(i)
                        .col(0)
                        .value()
                    == 0.0f)
                {
                    this->localizationCategory_.row(i).col(0) = Matrix::Zero(1, 1);
                }
                else
                {
                    this->localizationCategory_.row(i).col(0) = Matrix::Identity(1, 1) * 0.5f;
                }
            }
            else
            {
                this->localizationCategory_.row(i).col(0) = Matrix::Identity(1, 1);
            }
        }

        // To Pharos categorization. (Translation)
        for (size_t i = 0; i < 3; i++)
        {
            if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(i).col(0).value()
                == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
            {
                if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                        .translationConstraintValues_.row(i)
                        .col(0)
                        .value()
                    == 0.0f)
                {
                    this->localizationCategory_.row(i + 3).col(0) = Matrix::Zero(1, 1);
                }
                else
                {
                    this->localizationCategory_.row(i + 3).col(0) = Matrix::Identity(1, 1) * 0.5f;
                }
            }
            else
            {
                this->localizationCategory_.row(i + 3).col(0) = Matrix::Identity(1, 1);
            }
        }

        //std::cout << "Registered categories " << std::endl;
        //std::cout << "degenerateDirectionsInMapFrame_: " << this->degenerateDirectionsInMapFrame_ << std::endl;

        // Test whether to continue ICP.
        try
        {
            this->transformationCheckers.check(T_iter, iterate);
        }
        catch (const typename TransformationCheckersImpl<T>::CounterTransformationChecker::MaxNumIterationsReached& e)
        {
            iterate = false;
            this->maxNumIterationsReached = true;
        }

        T scalabilityStudyInMiliSeconds = 1000.0f * scalabilityStudy.elapsed();

        NbMatchesVet.emplace_back(this->localizabilityDetectionParameters.numberOfPoints);
        scalabilityVect.emplace_back(scalabilityStudyInMiliSeconds);

        //this->errorMinimizer->appendTransformation(T_refIn_refMean * T_iter * T_refIn_refMean.inverse() * T_refIn_dataIn);
        //this->errorMinimizer->appendErrorElements(this->errorMinimizer->getErrorElements());

        // std::cout << "scalabilityStudy: " << scalabilityStudyInMiliSeconds << " NbMatches: " << this->localizabilityDetectionParameters.numberOfPoints << std::endl;

        ++iterationCount;
    }

    this->inspector->addStat("IterationsCount", iterationCount);
    this->inspector->addStat("PointCountTouched", this->matcher->getVisitCount());
    this->matcher->resetVisitCount();
    this->inspector->addStat("OverlapRatio", this->errorMinimizer->getWeightedPointUsedRatio());
    this->inspector->addStat("ConvergenceDuration", t.elapsed());
    this->inspector->finish(iterationCount);

    LOG_INFO_STREAM("PointMatcher::icp - " << iterationCount << " iterations took " << t.elapsed() << " [s]");

    // Move transformation back to original coordinate (without center of mass)
    // T_iter is equivalent to: T_iter(i+1)_iter(0)
    // the frame <iter(0)> equals <refMean>
    // so we have:
    //   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_refMean_dataIn
    //   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_iter(0)_dataIn
    // T_refIn_refMean remove the temperary frame added during initialization

    T timingSum = std::accumulate(scalabilityVect.begin(), scalabilityVect.end(), 0.0);
    T nbMatchesSum = std::accumulate(NbMatchesVet.begin(), NbMatchesVet.end(), 0.0);
    //std::cout << "Mean Extra Scalability Time: " << timingSum / scalabilityVect.size() << " Mean Matches Per Iteration: " << int(nbMatchesSum / NbMatchesVet.size()) << " [s]" << std::endl;

    TransformationParameters icpLocalizationInMap;
    if (localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_)
    {
        MELO_WARN_STREAM("Pointmatcher ICP: Returning prior, ICP was not successful. ");
        icpLocalizationInMap = T_refIn_dataIn;
    }
    else
    {
        icpLocalizationInMap = T_refIn_refMean * T_iter * T_refIn_refMean.inverse() * T_refIn_dataIn;
    }

    return (icpLocalizationInMap);
}

template struct PointMatcher<float>::ICP;
//template struct PointMatcher<double>::ICP;


//! Return whether the object currently holds a valid map
template<typename T>
bool PointMatcher<T>::ICPSequence::hasMap() const
{
    return (mapPointCloud.features.cols() != 0);
}

//! Set the map using inputCloud
template<typename T>
bool PointMatcher<T>::ICPSequence::setMap(const DataPoints& inputCloud)
{
    // Ensuring minimum definition of components
    if (!this->matcher)
        throw runtime_error("You must setup a matcher before running ICP");
    if (!this->inspector)
        throw runtime_error("You must setup an inspector before running ICP");

    timer t; // Print how long take the algo
    const int dim(inputCloud.features.rows());
    const int ptCount(inputCloud.features.cols());

    // update keyframe
    if (ptCount == 0)
    {
        LOG_WARNING_STREAM("Ignoring attempt to create a map from an empty cloud");
        return false;
    }

    this->inspector->addStat("MapPointCount", inputCloud.features.cols());

    // Set map
    mapPointCloud = inputCloud;

    // Create intermediate frame at the center of mass of reference pts cloud
    //  this help to solve for rotations
    const Vector meanMap = mapPointCloud.features.rowwise().sum() / ptCount;
    this->T_refIn_refMean = Matrix::Identity(dim, dim);
    this->T_refIn_refMean.block(0, dim - 1, dim - 1, 1) = meanMap.head(dim - 1);

    // Reajust reference position (only translations):
    // from here reference is express in frame <refMean>
    // Shortcut to do T_refIn_refMean.inverse() * reference
    mapPointCloud.features.topRows(dim - 1).colwise() -= meanMap.head(dim - 1);

    // Apply reference filters
    this->referenceDataPointsFilters.init();
    this->referenceDataPointsFilters.apply(mapPointCloud);

    this->matcher->init(mapPointCloud);

    this->inspector->addStat("SetMapDuration", t.elapsed());

    return true;
}

//! Clear the map (reset to same state as after the object is created)
template<typename T>
void PointMatcher<T>::ICPSequence::clearMap()
{
    const int dim(mapPointCloud.features.rows());
    this->T_refIn_refMean = Matrix::Identity(dim, dim);
    mapPointCloud = DataPoints();
}

template<typename T>
void PointMatcher<T>::ICPSequence::setDefault()
{
    ICPChainBase::setDefault();

    if (mapPointCloud.getNbPoints() > 0)
    {
        this->matcher->init(mapPointCloud);
    }
}

template<typename T>
void PointMatcher<T>::ICPSequence::loadFromYaml(std::istream& in)
{
    ICPChainBase::loadFromYaml(in);

    if (mapPointCloud.getNbPoints() > 0)
    {
        this->matcher->init(mapPointCloud);
    }
}

//! Return the map, in global coordinates (slow)
template<typename T>
const typename PointMatcher<T>::DataPoints PointMatcher<T>::ICPSequence::getPrefilteredMap() const
{
    DataPoints globalMap(mapPointCloud);
    if (this->hasMap())
    {
        const int dim(mapPointCloud.features.rows());
        const Vector meanMapNonHomo(this->T_refIn_refMean.block(0, dim - 1, dim - 1, 1));
        globalMap.features.topRows(dim - 1).colwise() += meanMapNonHomo;
    }

    return globalMap;
}

//! Return the map, in global coordinates (slow). Deprecated in favor of getPrefilteredMap()
template<typename T>
const typename PointMatcher<T>::DataPoints PointMatcher<T>::ICPSequence::getMap() const
{
    return PointMatcher<T>::ICPSequence::getPrefilteredMap();
}

//! Return the map, in internal coordinates (fast)
template<typename T>
const typename PointMatcher<T>::DataPoints& PointMatcher<T>::ICPSequence::getPrefilteredInternalMap() const
{
    return mapPointCloud;
}

//! Return the map, in internal coordinates (fast). Deprecated in favor of getPrefilteredInternalMap().
template<typename T>
const typename PointMatcher<T>::DataPoints& PointMatcher<T>::ICPSequence::getInternalMap() const
{
    return PointMatcher<T>::ICPSequence::getPrefilteredInternalMap();
}

//! Apply ICP to cloud cloudIn, with identity as initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::operator()(const DataPoints& cloudIn)
{
    const int dim = cloudIn.features.rows();
    const TransformationParameters identity = TransformationParameters::Identity(dim, dim);
    return this->compute(cloudIn, identity);
}

//! Apply ICP to cloud cloudIn, with initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::operator()(
    const DataPoints& cloudIn, const TransformationParameters& T_dataInOld_dataInNew)
{
    return this->compute(cloudIn, T_dataInOld_dataInNew);
}

//! Apply ICP to cloud cloudIn, with initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::compute(const DataPoints& cloudIn,
                                                                                         const TransformationParameters& T_refIn_dataIn)
{
    // initial keyframe
    if (!hasMap())
    {
        const int dim(cloudIn.features.rows());
        LOG_WARNING_STREAM("Ignoring attempt to perform ICP with an empty map");
        return Matrix::Identity(dim, dim);
    }

    this->inspector->init();

    return this->computeWithTransformedReference(cloudIn, mapPointCloud, this->T_refIn_refMean, T_refIn_dataIn);
}

template<typename T>
void PointMatcher<T>::ICP::calculateOptimizationHessian(
    Eigen::Matrix<T, 6, 6>& hessian, Eigen::Matrix<T, 6, 1>& constraints, ErrorElements& matches,
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    MELO_DEBUG_STREAM("Step-(1) Calculating the optimization variables.");

    BOOST_AUTO(referenceSurfaceNormals, matches.reference.getDescriptorViewByName("normals"));

    // Need to have proper normals.
    assert(referenceSurfaceNormals.rows() = 3);
    // Cross is the cross product elements used for rotation subspace.
    // wF is the weighted feature matrix. 6xN
    // F is the regular feature matrix. 6xN
    Matrix cross = this->errorMinimizer->crossProduct(matches.reading.features, referenceSurfaceNormals);
    Matrix wF(referenceSurfaceNormals.rows() + cross.rows(), referenceSurfaceNormals.cols());
    Matrix F(referenceSurfaceNormals.rows() + cross.rows(), referenceSurfaceNormals.cols());

    // Regular point to plane ICP cost. Directly re-used from libpointmatcher implementation.
    for (int i = 0; i < cross.rows(); i++)
    {
        //wF.row(i) = matches.weights.array() * cross.row(i).array();
        F.row(i) = cross.row(i);
    }
    for (int i = 0; i < referenceSurfaceNormals.rows(); i++)
    {
        //wF.row(i + cross.rows()) = matches.weights.array() * referenceSurfaceNormals.row(i).array();
        F.row(i + cross.rows()) = referenceSurfaceNormals.row(i);
    }
    // Unadjust covariance A = wF * F'
    hessian = F * F.transpose();

    if (this->localizabilityDetectionParameters.isDebugModeENabled_)
    {
        //PointToPlaneErrorMinimizer::setCovarianceMatrixTurcan(hessian.topLeftCorner(6, 6));
        this->errorMinimizer->setCovarianceMatrixTurcan(hessian.topLeftCorner(6, 6));
    }

    // Compute the differences between reading and reference matched points.
    const Matrix deltas = matches.reading.features - matches.reference.features;

    // Save the differences in points from reading and reference (delta)
    localizabilityParametersForErrorMinimization.deltas_ = deltas;

    // dot product of dot = dot(deltas, normals)
    Matrix dotProd = Matrix::Zero(1, referenceSurfaceNormals.cols());

    for (int i = 0; i < referenceSurfaceNormals.rows(); i++)
    {
        dotProd += (deltas.row(i).array() * referenceSurfaceNormals.row(i).array()).matrix();
    }
    // b = -(wF' * dot)
    constraints = -(F * dotProd.transpose());
}

template<typename T>
void PointMatcher<T>::ICP::eigenAnalysis(const Eigen::Matrix<T, 6, 6>& hessian, Eigen::Matrix<T, 6, 6>& eigenvectors66,
                                         Eigen::Matrix<T, 6, 1>& eigenvalues6)
{
    MELO_DEBUG_STREAM("Doing 6x6 eigenanalysis.");
    // Does simple eigenAnalysis with SVD for a 6x6 matrix.
    Eigen::JacobiSVD<Eigen::Matrix<T, 6, 6>> svd_all(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    eigenvalues6 = svd_all.singularValues();
    eigenvectors66 = svd_all.matrixU();
}

template<typename T>
void PointMatcher<T>::ICP::eigenAnalysis(const Eigen::Matrix<T, 6, 6>& hessian, Eigen::Matrix<T, 3, 3>& translationEigenvectors33,
                                         Eigen::Matrix<T, 3, 3>& rotationEigenvectors33)
{
    MELO_DEBUG_STREAM("Step-(2) Doing 3x3 eigenAnalysis.");
    // Does simple eigenAnalysis with SVD for a 3x3 matrix.
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_bottomRightCorner(hessian.bottomRightCorner(3, 3),
                                                                   Eigen::ComputeFullU | Eigen::ComputeFullV);
    translationEigenvectors33 = svd_bottomRightCorner.matrixU();

    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_topLeftCorner(hessian.topLeftCorner(3, 3), Eigen::ComputeFullU | Eigen::ComputeFullV);
    rotationEigenvectors33 = svd_topLeftCorner.matrixU();
}

template<typename T>
void PointMatcher<T>::ICP::calculatePointCloudCenter(Eigen::Matrix<T, 3, 1>& center, const Matrix& features,
                                                     const Eigen::Index numberOfPoints)
{
    MELO_DEBUG_STREAM("Calculating point cloud center.");
    // If operations are in 3D space. featDim is 4 for ease of point transformation.
    const Eigen::Index featDim(features.rows());

    //Compute centroid
    for (Eigen::Index i = 0; i < featDim - 1; ++i)
    {
        center(i) = T(0.);
    }

    for (Eigen::Index i = 0; i < numberOfPoints; ++i)
    {
        for (Eigen::Index f = 0; f <= 3; ++f)
        {
            center(f) += features(f, i);
        }
    }

    for (Eigen::Index i = 0; i <= 3; ++i)
    {
        center(i) /= T(numberOfPoints);
    }
}

template<typename T>
void PointMatcher<T>::ICP::solutionRemappingProjectionCalculation(Matrix& projectionMatrix, const Matrix& eigenvectors,
                                                                  const Vector& eigenvalues, const T& threshold)
{
    if(this->localizabilityDetectionParameters.isPrintingEnabled_){
    MELO_INFO_STREAM("Solution Remapping projection matrix calculation.");
    }
    bool isDegenerate{ false };
    Eigen::Matrix<T, 6, 6> eigenvectorsCopy{ eigenvectors };
    std::vector<T> eigenvalueThresholds(6, threshold);

    // Care order of eigenvectors and eigenvalues between my detection and here. Maybe use SVD here as well?
    for (Eigen::Index j = 0; j < eigenvectors.cols(); j++)
    {
        if (eigenvalues.row(0).col(j).value() < eigenvalueThresholds[j])
        {
            this->solRemapCategories.row(j).col(0) = Matrix::Zero(1, 1);
            if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM(message_logger::color::blue << "Eigvalue: " << eigenvalues.row(0).col(j).value() << " / "
                                                         << eigenvalueThresholds[j]);
            MELO_INFO_STREAM(message_logger::color::blue << "EigVector: " << eigenvectorsCopy.col(j).transpose());
            }
            if(this->degeneracySolverOptions_.useSixDofRegularization_){
                std::cout << "Using SIXDOF REGULARIZATION" << std::endl;
                this->ceresEigVect_.col(j) = eigenvectorsCopy.col(j);
            }
            eigenvectorsCopy.col(j) = Eigen::Matrix<T, 6, 1>::Zero(6, 1);
            isDegenerate = true;
            if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM(message_logger::color::blue << "Solution Remapping found a degenerate direction_1.");
            }
        }
        else
        {
            this->solRemapCategories.row(j).col(0) = Matrix::Identity(1, 1);
        }
    }

    if (isDegenerate)
    {
        if(this->localizabilityDetectionParameters.isPrintingEnabled_){
        MELO_INFO_STREAM(message_logger::color::blue << "MAIN_Solution Remapping found a degenerate direction.");
        }
        projectionMatrix = eigenvectors.transpose().inverse() * eigenvectorsCopy.transpose();
    }
}

template<typename T>
void PointMatcher<T>::ICP::calculateCrossProductElements(Matrix& crosses, const Matrix& normals, const Matrix& features,
                                                         const Eigen::Index& numberOfPoints)
{
    MELO_DEBUG_STREAM("Step-(4). Calculate cross product elements.");

    // Calculate the center of given features.
    Eigen::Matrix<T, 3, 1> center = Vector::Zero(3, 1);
    calculatePointCloudCenter(center, features, numberOfPoints);

    // Create container feature labels.
    Labels featLabels;
    featLabels.emplace_back(Label("x", 1));
    featLabels.emplace_back(Label("y", 1));
    featLabels.emplace_back(Label("z", 1));
    featLabels.emplace_back(Label("pad", 1));

    // Descriptors.
    const Labels descLabels;

    // Create empty similar point cloud.
    DataPoints dataPointsWithNorm = DataPoints(featLabels, descLabels, numberOfPoints);
    for (Eigen::Index i = 0; i < numberOfPoints; ++i)
    {
        dataPointsWithNorm.features.col(i).head(3) = features.col(i).head(3) - center;
    }
    crosses = this->errorMinimizer->crossProduct(dataPointsWithNorm.features, normals);
}

template<typename T>
bool PointMatcher<T>::ICP::detectLocalizabilityWithTernaryLevelDetection(
    ErrorElements& matchedPoints, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    // If there was no degeneracy at the first iteration, we don't need to repeat the analysis.
    localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_ = true;
    if (!localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_)
    {
        return false;
    }

    // Eigenanalysis. Since the data in the hessian was in mean substracted map frame, the eigenvectors are also defined in this frame.
    eigenAnalysis(localizabilityParametersForErrorMinimization.A_,
                  localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_,
                  localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_);

    // Assign reference normals to the reading point cloud since if we are going to sample, we sample that information together.
    {
        BOOST_AUTO(normals, matchedPoints.reference.getDescriptorViewByName("normals"));
        matchedPoints.reading.addDescriptor("matched_reference_normals", normals);
        matchedPoints.reading.addDescriptor("delta_features", localizabilityParametersForErrorMinimization.deltas_);
    }

    // This transformation reverts the transformation coming from the inital guess thus, converting to the the data to the frame it came from (base frame.)
    this->transformations.apply(matchedPoints.reading, this->localizabilityDetectionParameters.transformationToOptimizationFrame.inverse());

    // YENI EKLEDIM
    this->transformations.apply(matchedPoints.reading,
                                localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.inverse());

    MELO_DEBUG_STREAM("Step-(3) Preparation. Transform information to the base frame.");

    // Need to get the refence again after the rotation which is done in place.
    BOOST_AUTO(normalsRef, matchedPoints.reading.getDescriptorViewByName("matched_reference_normals"));

    // Rotate the eigenvector to the base frame. From the mean substracted map frame.
    for (Eigen::Index i = 0; i < 3; i++)
    {
        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);
    }

    MELO_DEBUG_STREAM("Step-(3) Preparation. Rotated eigenvectors to the base frame.");

    // Cross product calculation is required for rotation alignment vector calculation.
    Matrix crosses = Matrix::Zero(3, this->localizabilityDetectionParameters.numberOfPoints);
    calculateCrossProductElements(
        crosses, normalsRef, matchedPoints.reading.features, this->localizabilityDetectionParameters.numberOfPoints);

    // Allocate alignment vectors.
    AlignmentVector translationAlignmentVector;
    translationAlignmentVector.resize(this->localizabilityDetectionParameters.numberOfPoints);
    AlignmentVector rotationAlignmentVector;
    rotationAlignmentVector.resize(this->localizabilityDetectionParameters.numberOfPoints);

    // Populate the alignment vectors.
    for (Eigen::Index i = 0; i < this->localizabilityDetectionParameters.numberOfPoints; ++i)
    {
        // Fill up alinment vectors.
        translationAlignmentVector[i].template block<3, 1>(0, 0) = normalsRef.col(i).head(3);
        if (crosses.col(i).norm() < 1.0f)
        {
            //std::cout << " SMOLL " << std::endl;
            rotationAlignmentVector[i].template block<3, 1>(0, 0) = crosses.col(i).head(3);
        }
        else
        {
            rotationAlignmentVector[i].template block<3, 1>(0, 0) = crosses.col(i).head(3).normalized();
        }
    }

    MELO_DEBUG_STREAM("Step-(5). Calculated the alignment vectors.");

    // Iterate through all eigenvectors. A subspace (translation or rotation) contains 3 eigenvectors.
    for (Eigen::Index eigenvectorIndex = 0; eigenvectorIndex < 3; ++eigenvectorIndex)
    {
        // Detection for rotation subspace.
        const Eigen::Matrix<T, 3, 1>& rotationEigenVector =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.template block<3, 1>(
                0, eigenvectorIndex);
        MELO_DEBUG_STREAM("Rotation Eigenvector of interest: " << rotationEigenVector.transpose());
        detectSubspaceLocalizability(matchedPoints.reading,
                                     localizabilityParametersForErrorMinimization,
                                     rotationEigenVector,
                                     rotationAlignmentVector,
                                     eigenvectorIndex,
                                     true);

        this->totalContribution_.row(eigenvectorIndex).col(0) << this->localizabilityDetectionParameters.combinedContribution;
        this->highContribution_.row(eigenvectorIndex).col(0) << this->localizabilityDetectionParameters.highContribution;

        // Get all the contributions
        for (std::size_t i = 0; i < this->nbMatches_; ++i)
        {
            const T contElement = this->localizabilityDetectionParameters.samplingAlignmentList[i].second;
            this->plottingDistributionContribution_.row(i).col(eigenvectorIndex) << contElement;
        }

        // The eigenvector
        this->eigenVectorsDistribution_.col(eigenvectorIndex) = rotationEigenVector;


        // Cleanup
        this->localizabilityDetectionParameters.contributingNumberOfPoints = 0u;
        this->localizabilityDetectionParameters.highlyContributingNumberOfPoints = 0u;
        this->localizabilityDetectionParameters.combinedContribution = 0.0f;
        this->localizabilityDetectionParameters.highContribution = 0.0f;
        this->localizabilityDetectionParameters.samplingAlignmentList.clear();

        // Detection for translation subspace.
        const Eigen::Matrix<T, 3, 1>& translationEigenVector =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.template block<3, 1>(
                0, eigenvectorIndex);
        MELO_DEBUG_STREAM("Translation Eigenvector of interest: " << translationEigenVector.transpose());
        detectSubspaceLocalizability(matchedPoints.reading,
                                     localizabilityParametersForErrorMinimization,
                                     translationEigenVector,
                                     translationAlignmentVector,
                                     eigenvectorIndex,
                                     false);

        this->totalContribution_.row(eigenvectorIndex + 3).col(0) << this->localizabilityDetectionParameters.combinedContribution;
        this->highContribution_.row(eigenvectorIndex + 3).col(0) << this->localizabilityDetectionParameters.highContribution;

        // Get all the contributions
        for (std::size_t i = 0; i < this->nbMatches_; ++i)
        {
            const T contElement = this->localizabilityDetectionParameters.samplingAlignmentList[i].second;
            this->plottingDistributionContribution_.row(i).col(eigenvectorIndex + 3) << contElement;
        }

        // The eigenvector
        this->eigenVectorsDistribution_.col(eigenvectorIndex + 3) = translationEigenVector;

        // Make the data globally available if its the last eigenvector.
        if (eigenvectorIndex == 2)
        {
            this->contributionDistributionPlotting_ = this->plottingDistributionContribution_;
        }

        // Cleanup
        this->localizabilityDetectionParameters.contributingNumberOfPoints = 0u;
        this->localizabilityDetectionParameters.highlyContributingNumberOfPoints = 0u;
        this->localizabilityDetectionParameters.combinedContribution = 0.0f;
        this->localizabilityDetectionParameters.highContribution = 0.0f;
        this->localizabilityDetectionParameters.samplingAlignmentList.clear();
    }

    // Rotate the eigenvectors back to map frame.
    //(TODO) Can't we just multiply as matrix?
    for (Eigen::Index i = 0; i < 3; i++)
    {

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);
    }

    // Check if there is a degenerate direction. If there is no degenerate direction, then we won't do detection for the following iterations.
    // Set the repeatition to false, only re-activate if a direction is nonLocalizable.
    //localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_ = false;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        const T degeneracyLevelXYZ =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(i).col(0).value();
        const T degeneracyLevelRPY =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(i).col(0).value();

        if ((degeneracyLevelXYZ == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
            || (degeneracyLevelRPY == static_cast<T>(LocalizabilityCategory::kNonLocalizable)))
        {
            localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_ = true;
            break;
        }
    }

    if (!localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_)
    {
        MELO_DEBUG_STREAM("All directions are localizable. Will not repeat localizability analysis.");
    }
    return true;
}

template<typename T>
void PointMatcher<T>::ICP::detectSubspaceLocalizability(
    const DataPoints& inputCloud, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization,
    const Vector& eigenvector, const AlignmentVector& alignmentVector, const Eigen::Index index, const bool isRotationSubspace)
{
    MELO_DEBUG_STREAM("Step-(6). Detect subspace Localizability. Index: " << index << " Is rotation: " << isRotationSubspace
                                                                          << " and eigenvector: " << eigenvector.transpose());

    // Decide the localizability level here.
    LocalizabilitySamplingType localizabilitySamplingType{ decideLocalizabilityLevel(
        localizabilityParametersForErrorMinimization, eigenvector, alignmentVector, index, isRotationSubspace) };
    Eigen::Index totalNumberOfPointsToSample{ 0u };

    // OVERWRITE PARTIALITY FOR DEBUGGING
    //localizabilitySamplingType = LocalizabilitySamplingType::kInsufficientPoints;

    switch (localizabilitySamplingType)
    {
        case (LocalizabilitySamplingType::kHighContributionPoints): {
            MELO_DEBUG_STREAM("Minimal Subset Partial localizability type. Re-sampling will be done.");
            totalNumberOfPointsToSample = this->localizabilityDetectionParameters.highlyContributingNumberOfPoints;
        }
        break;
        case (LocalizabilitySamplingType::kMixedContributionPoints): {
            MELO_DEBUG_STREAM("Combined Partial localizability type. Re-sampling will be done.");
            totalNumberOfPointsToSample = this->localizabilityDetectionParameters.contributingNumberOfPoints;
        }
        break;
        case (LocalizabilitySamplingType::kUnnecessary): {
            MELO_DEBUG_STREAM("Localizable. Skipped re-sampling for iteration, no reason to resample.");
            return;
        }
        break;
        case (LocalizabilitySamplingType::kInsufficientPoints): {
            MELO_DEBUG_STREAM("Non-Localizable thus skipped re-sampling.");
            return;
        }
        break;
        default: {
            // This shouldn't happen. Both of the flags can't be true. Thus we will return the prior.
            localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_ = true;
            MELO_ERROR_STREAM("Localizability type detection failed. ICP won't be completed.");
            return;
        }
        break;
    }

    // Check the sanity of the points to sample.
    bool isDataSane{ (totalNumberOfPointsToSample >= this->localizabilityDetectionParameters.insufficientInformationThreshold) };
    isDataSane &= (this->localizabilityDetectionParameters.numberOfPoints >= totalNumberOfPointsToSample);

    if (!isDataSane)
    {
        MELO_ERROR_STREAM("Sampling data is not sane. Total Points:  "
                          << this->localizabilityDetectionParameters.numberOfPoints
                          << " Number of points to sample: " << totalNumberOfPointsToSample << " / "
                          << this->localizabilityDetectionParameters.insufficientInformationThreshold << ". Points won't be sampled.");
        localizabilityParametersForErrorMinimization.debugging_.whetherToReturnPrior_ = true;
        return;
    }

    // Create an empty point cloud with the size we want to sample.
    DataPoints sampledCloud(inputCloud.createSimilarEmpty(totalNumberOfPointsToSample));
    samplePointCloud(inputCloud,
                     sampledCloud,
                     this->localizabilityDetectionParameters.samplingAlignmentList,
                     totalNumberOfPointsToSample,
                     this->localizabilityDetectionParameters.numberOfPoints);

    // Moving the sampled point cloud back to the latest mean extracted map frame. Since this is a copy, has no effect on the original inputCloud.
    this->transformations.apply(sampledCloud, localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat);
    this->transformations.apply(sampledCloud, this->localizabilityDetectionParameters.transformationToOptimizationFrame);

    // Solve a simplified least squares problem and assign the constraints to the optiization struct.
    solveSimpleOptimizationProblemForPartialConstraints(localizabilityParametersForErrorMinimization,
                                                        sampledCloud,
                                                        isRotationSubspace,
                                                        index,
                                                        this->localizabilityDetectionParameters.transformationToOptimizationFrame);
}

template<typename T>
typename PointMatcher<T>::LocalizabilitySamplingType PointMatcher<T>::ICP::decideLocalizabilityLevel(
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization, const Vector& eigenvector,
    const AlignmentVector& alignmentVector, const Eigen::Index& eigenvectorIndex, const bool isRotationSubspace)
{
    if(this->localizabilityDetectionParameters.isPrintingEnabled_){
    MELO_INFO_STREAM("Step-(7). Decide the localizability level in ternary fashion.");
    }

    const std::string typeStr{ (!isRotationSubspace) ? "translation" : "rotation" };
    Vector& localLocalizability = (!isRotationSubspace)
        ? localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_
        : localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_;
    Vector& localConstraints = (!isRotationSubspace)
        ? localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
              .translationConstraintValues_
        : localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_.rotationConstraintValues_;

    // Sufficient information, either the combined contribution is bigger than the safety threshold or the higher contribution values are better than the localizability threshold.
    if (countContributionValuesAndCheckWhetherProblemIsConstrainedVeryWell(alignmentVector, eigenvector))
    {
        if(this->localizabilityDetectionParameters.isPrintingEnabled_){
        MELO_INFO_STREAM(message_logger::color::green << "Localizable: " << this->localizabilityDetectionParameters.combinedContribution
                                                      << " / " << this->localizabilityDetectionParameters.highInformationThreshold
                                                      << " and "
                                                      << " Upper: " << this->localizabilityDetectionParameters.highContribution << " / "
                                                      << this->localizabilityDetectionParameters.enoughInformationThreshold);
        MELO_INFO_STREAM(message_logger::color::green << "Eigenvector " << typeStr << " (In base frame): " << eigenvector.transpose());
        }
        localLocalizability.row(eigenvectorIndex).col(0) << static_cast<T>(LocalizabilityCategory::kLocalizable);
        return LocalizabilitySamplingType::kUnnecessary;
    }

    const bool partialLocalizabilityMixedContributionCondition{
        (this->localizabilityDetectionParameters.combinedContribution >= this->localizabilityDetectionParameters.enoughInformationThreshold)
        && (this->localizabilityDetectionParameters.combinedContribution < this->localizabilityDetectionParameters.highInformationThreshold)
    };
    // Partial localizability type 2 : combined. Here we capture that the combined information is bigger than the localizability threshold but not big enough to be safely considered to be localizable.
    if (partialLocalizabilityMixedContributionCondition)
    {
        if(this->localizabilityDetectionParameters.isPrintingEnabled_){
        // Warnings are throttled as a result in order to properly debug we also output debug level messages.
        MELO_INFO_STREAM(message_logger::color::yellow << "Partially Localizable with diverse contribution points: "
                                                       << this->localizabilityDetectionParameters.enoughInformationThreshold << " < "
                                                       << this->localizabilityDetectionParameters.combinedContribution << " < "
                                                       << this->localizabilityDetectionParameters.highInformationThreshold << " and Upper: "
                                                       << this->localizabilityDetectionParameters.insufficientInformationThreshold << " < "
                                                       << this->localizabilityDetectionParameters.highContribution << " < "
                                                       << this->localizabilityDetectionParameters.enoughInformationThreshold);
        MELO_INFO_STREAM(message_logger::color::yellow << "Eigenvector " << typeStr << " (In base frame) : " << eigenvector.transpose());
        }
        // Partial localizability is also captured under being non-localizable since its just being non-localizable with a non-zero constraint.
        localLocalizability.row(eigenvectorIndex).col(0) << static_cast<T>(LocalizabilityCategory::kNonLocalizable);

        return LocalizabilitySamplingType::kMixedContributionPoints;
    }

    // If you didn't return yet, one of the directions is degenerate. The rest of the function detects which level of degeneracy it is.
    const bool partialLocalizabilityHighContributionCondition{ (
        this->localizabilityDetectionParameters.highContribution
        >= this->localizabilityDetectionParameters.insufficientInformationThreshold) };

    // Partial localizability type 1 : minimal subset. Here we capture if the high contribution value is above the minimum threshold but not big enough to be considered safely localizable.
    if (partialLocalizabilityHighContributionCondition)
    {
        if(this->localizabilityDetectionParameters.isPrintingEnabled_){
        // Warnings are throttled as a result in order to properly debug we also output debug level messages.
        MELO_INFO_STREAM("Partially Localizable with high contribution points: "
                         << this->localizabilityDetectionParameters.combinedContribution << " / "
                         << this->localizabilityDetectionParameters.enoughInformationThreshold
                         << " and Upper: " << this->localizabilityDetectionParameters.enoughInformationThreshold << ">"
                         << this->localizabilityDetectionParameters.highContribution << " > "
                         << this->localizabilityDetectionParameters.insufficientInformationThreshold);
        MELO_INFO_STREAM("Eigenvector " << typeStr << " (In base frame) : " << eigenvector.transpose());
        }
        // Partial localizability is also captured under being non-localizable since its just being non-localizable with a non-zero constraint.
        localLocalizability.row(eigenvectorIndex).col(0) << static_cast<T>(LocalizabilityCategory::kNonLocalizable);
        return LocalizabilitySamplingType::kHighContributionPoints;
    }
    if(this->localizabilityDetectionParameters.isPrintingEnabled_){
    // Not enough information to localize.
    MELO_INFO_STREAM(message_logger::color::blue << "Non-Localizable: " << this->localizabilityDetectionParameters.combinedContribution
                                                 << " < " << this->localizabilityDetectionParameters.enoughInformationThreshold << " and "
                                                 << " Upper: " << this->localizabilityDetectionParameters.highContribution << " / "
                                                 << this->localizabilityDetectionParameters.insufficientInformationThreshold);
    MELO_INFO_STREAM(message_logger::color::blue << "Eigenvector " << typeStr << " (In base frame) : " << eigenvector.transpose());
    }
    // If we are not localizable then we assign the constraint as 0.0f which indicates that there shouldn't be any motion in this direction.
    localConstraints.row(eigenvectorIndex).col(0) << 0.0f;
    localLocalizability.row(eigenvectorIndex).col(0) << static_cast<T>(LocalizabilityCategory::kNonLocalizable);

    return LocalizabilitySamplingType::kInsufficientPoints;
}

template<typename T>
bool PointMatcher<T>::ICP::countContributionValuesAndCheckWhetherProblemIsConstrainedVeryWell(const AlignmentVector& alignmentVector,
                                                                                              const Vector& eigenvector)
{
    MELO_DEBUG_STREAM("Step-(8). Count contributions and check whether the problem is very well constrained.");
    // Alignment list contains pairs of (index, magnitude) contribution to an eigensvector.
    AlignmentList& alignmentList{ this->localizabilityDetectionParameters.samplingAlignmentList };
    alignmentList.resize(this->localizabilityDetectionParameters.numberOfPoints);

    // Go through the points to get the alignment list.
    bool informationIsEnough{ false };
    for (Eigen::Index pointIndex = 0; pointIndex < this->localizabilityDetectionParameters.numberOfPoints && !informationIsEnough;
         ++pointIndex)
    {
        alignmentList[pointIndex].first = pointIndex;
        alignmentList[pointIndex].second = std::fabs(alignmentVector[pointIndex].dot(eigenvector));

        // If the alignment is bigger than our higher alignment threshold add the contribution to the higher contribution sum.
        // Keep a count of how many points there are since it might be required for re-sampling.
        if (alignmentList[pointIndex].second > this->localizabilityDetectionParameters.point2NormalStrongAlignmentCosineThreshold)
        {
            this->localizabilityDetectionParameters.highContribution += alignmentList[pointIndex].second;
            this->localizabilityDetectionParameters.highlyContributingNumberOfPoints++;
        }

        // If the alignment is bigger than our normal alignment threshold add the contribution to the combined contribution sum.
        // The higher contribution sum is a subset of the combined contribution sum.
        // Keep a count of how many points there are since it might be required for re-sampling.
        if (alignmentList[pointIndex].second >= this->localizabilityDetectionParameters.point2NormalMinimalAlignmentCosineThreshold)
        {
            this->localizabilityDetectionParameters.combinedContribution += alignmentList[pointIndex].second;
            this->localizabilityDetectionParameters.contributingNumberOfPoints++;
        }

        // The final decision is made by checking whether there is enough information within the sums. Observe that this boolean is also a condition of the for loop.
        // If you want to record all contribution you need to put this out of the loop.
        informationIsEnough = (this->localizabilityDetectionParameters.combinedContribution
                               >= this->localizabilityDetectionParameters.highInformationThreshold)
            || (this->localizabilityDetectionParameters.highContribution
                >= this->localizabilityDetectionParameters.enoughInformationThreshold);
    }
    return informationIsEnough;
}

template<typename T>
bool PointMatcher<T>::ICP::detectLocalizability(T& combinedContribution, T& highContribution, const AlignmentVector& alignmentVector,
                                                const Vector& eigenvector)
{
    bool informationIsEnough{ false };
    for (Eigen::Index pointIndex = 0; pointIndex < this->localizabilityDetectionParameters.numberOfPoints && !informationIsEnough;
         ++pointIndex)
    {
        // Calculate the alignment value.
        const T alignmentValue{ std::fabs(alignmentVector[pointIndex].dot(eigenvector)) };

        if (alignmentValue > this->localizabilityDetectionParameters.point2NormalMinimalAlignmentCosineThreshold)
        {
            combinedContribution += alignmentValue;
        }

        if (alignmentValue > this->localizabilityDetectionParameters.point2NormalStrongAlignmentCosineThreshold)
        {
            highContribution += alignmentValue;
        }

        // Check whether we accumulated enough information.
        // If you want to record all contribution you need to put this out of the loop.
        informationIsEnough = ((combinedContribution >= this->localizabilityDetectionParameters.enoughInformationThreshold)
                               || (highContribution >= this->localizabilityDetectionParameters.insufficientInformationThreshold));
    }

    return informationIsEnough;
}

template<typename T>
void PointMatcher<T>::ICP::samplePointCloud(const DataPoints& cloud, DataPoints& sampledCloud, AlignmentList& samplingAlignmentList,
                                            const Eigen::Index& totalNumberOfPointsToSample, const Eigen::Index& numberOfPoints)
{
    MELO_DEBUG_STREAM("Solving a simplified problem by sampling the point cloud");
    MELO_DEBUG_STREAM("Total points at hand: " << numberOfPoints << ". Number of points to sample: " << totalNumberOfPointsToSample);

    // Initialie the vector to keep the saved indices with a fixed size.
    std::vector<Eigen::Index> indexesToSave;
    indexesToSave.resize(totalNumberOfPointsToSample);

    // Partially sort the alignment list to get the Top N best contributors.
    std::partial_sort(samplingAlignmentList.begin(),
                      samplingAlignmentList.begin() + totalNumberOfPointsToSample,
                      samplingAlignmentList.end(),
                      compareAlignmentList);

    for (Eigen::Index savedPointIndex = 0; savedPointIndex < totalNumberOfPointsToSample; ++savedPointIndex)
    {
        // Get the index of the maximum contribution.
        const Eigen::Index maximumElementIndex{ samplingAlignmentList[savedPointIndex].first };

        //std::cout << "Loop index: " << savedPointIndex << "/" << totalNumberOfPointsToSample << " Max element index: " << maximumElementIndex << " Value: " << samplingAlignmentList[savedPointIndex].second << std::endl;

        sampledCloud.features.col(savedPointIndex) = cloud.features.col(maximumElementIndex);
        sampledCloud.descriptors.col(savedPointIndex) = cloud.descriptors.col(maximumElementIndex);
    }
}

template<typename T>
bool PointMatcher<T>::ICP::detectLocalizabilityWithOptimizedMethod(
    ErrorElements& matchedPoints, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    MELO_ERROR_STREAM("#############3 detectLocalizabilityWithOptimizedMethod ############");

    if (this->localizabilityDetectionParameters.numberOfPoints == 0u)
    {
        MELO_ERROR_STREAM("No points exists in the matcher This is not expected.");
        return false;
    }

    if (localizabilityParametersForErrorMinimization.A_ == Eigen::Matrix<T, 6, 6>::Zero(6, 6))
    {
        MELO_ERROR_STREAM("Optimization hessian is zero. This is not expected.");
        return false;
    }

    // Do eigenanalysis if it's the first iteration OR we detected degeneracy and we need to keep track of eigenvectors.
    // Here last eigenvector will have the smallest eigenvalue. Each column is an eigenvector.
    if (localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_)
    {
        eigenAnalysis(localizabilityParametersForErrorMinimization.A_,
                      localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_,
                      localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_);
    }

    if ((localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_
         == Eigen::Matrix<T, 3, 3>::Zero(3, 3))
        || (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_
            == Eigen::Matrix<T, 3, 3>::Zero(3, 3)))
    {
        MELO_ERROR_STREAM("Eigenvectors are zero. This is not expected.");
        return false;
    }

    if (!localizabilityParametersForErrorMinimization.debugging_.isItFirstIteration_)
    {
        MELO_DEBUG_STREAM("Early return from optimized localizability detection since this is not the first iteration.");
        // This is expected behavior, return true.
        return true;
    }

    // Assign reference normals to the reading point cloud since if we are going to sample, we sample that information together.
    {
        BOOST_AUTO(normals, matchedPoints.reference.getDescriptorViewByName("normals"));
        matchedPoints.reading.addDescriptor("matched_reference_normals", normals);
        //matchedPoints.reading.addDescriptor("delta_features", localizabilityParametersForErrorMinimization.deltas_);
    }

    // This transformation reverts the transformation coming from the inital guess thus, converting to the the data to the frame it came from (base frame.)
    this->transformations.apply(matchedPoints.reading, this->localizabilityDetectionParameters.transformationToOptimizationFrame.inverse());

    // YENI EKLEDIM
    this->transformations.apply(matchedPoints.reading,
                                localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.inverse());

    MELO_ERROR_STREAM("Step-(3) Preparation. Transform information to the base frame.");

    // Need to get the refence again after the rotation which is done in place.
    BOOST_AUTO(normalsRef, matchedPoints.reading.getDescriptorViewByName("matched_reference_normals"));

    // Rotate the eigenvector to the base frame. From the mean substracted map frame.
    for (Eigen::Index i = 0; i < 3; i++)
    {
        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3).inverse()
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);
    }

    // Calculate mean centered cross products.
    Matrix crosses = Matrix::Zero(3, this->localizabilityDetectionParameters.numberOfPoints);
    calculateCrossProductElements(
        crosses, normalsRef, matchedPoints.reading.features, this->localizabilityDetectionParameters.numberOfPoints);

    // Allocate memory for the alignment vectors.
    AlignmentVector translationAlignmentVector;
    translationAlignmentVector.resize(this->localizabilityDetectionParameters.numberOfPoints);
    AlignmentVector rotationAlignmentVector;
    rotationAlignmentVector.resize(this->localizabilityDetectionParameters.numberOfPoints);

    for (Eigen::Index i = 0; i < this->localizabilityDetectionParameters.numberOfPoints; ++i)
    {
        // Fill up alinment vectors.
        translationAlignmentVector[i].template block<3, 1>(0, 0) = normalsRef.col(i).head(3);
        //rotationAlignmentVector[i].template block<3, 1>(0, 0) = crosses.col(i).head(3).normalized();
        if (crosses.col(i).norm() < 1.0f)
        {
            rotationAlignmentVector[i].template block<3, 1>(0, 0) = crosses.col(i).head(3);
        }
        else
        {
            rotationAlignmentVector[i].template block<3, 1>(0, 0) = crosses.col(i).head(3).normalized();
        }
    }

    // Iterate for each eigenvector.
    for (Eigen::Index eigenvectorIndex = 0; eigenvectorIndex < 3; ++eigenvectorIndex)
    {
        // Regular Summation
        T sumOfLowerAndHigherAlignmentRegion{ 0.0f };
        T sumOfHigherAlignmentRegion{ 0.0f };

        const Eigen::Matrix<T, 3, 1>& translationEigenVector =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.template block<3, 1>(
                0, eigenvectorIndex);
        // Set flags (Translation) (this is te heaviest function to compute.)
        if (detectLocalizability(
                sumOfLowerAndHigherAlignmentRegion, sumOfHigherAlignmentRegion, translationAlignmentVector, translationEigenVector))
        {
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(eigenvectorIndex).col(0)
                << static_cast<T>(LocalizabilityCategory::kLocalizable);
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                    .translationConstraintValues_.row(eigenvectorIndex)
                    .col(0)
                << 1.0f;
            if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM("Sufficient Information: " << sumOfLowerAndHigherAlignmentRegion << " / "
                                                        << this->localizabilityDetectionParameters.enoughInformationThreshold << " and "
                                                        << " Upper: " << sumOfHigherAlignmentRegion << " / "
                                                        << this->localizabilityDetectionParameters.insufficientInformationThreshold << "\n"
                                                        << "Translation Eigenvector Index " << eigenvectorIndex << " is "
                                                        << translationEigenVector.transpose() << "\n\n");
            }
        }
        else
        {
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(eigenvectorIndex).col(0)
                << static_cast<T>(LocalizabilityCategory::kNonLocalizable);
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                    .translationConstraintValues_.row(eigenvectorIndex)
                    .col(0)
                << 0.0f;
                if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM("Not Enough Information: " << sumOfLowerAndHigherAlignmentRegion << " < "
                                                        << this->localizabilityDetectionParameters.enoughInformationThreshold << " and "
                                                        << " Upper: " << sumOfHigherAlignmentRegion << " < "
                                                        << this->localizabilityDetectionParameters.insufficientInformationThreshold << "\n"
                                                        << "Translation Eigenvector Index " << eigenvectorIndex << " is "
                                                        << translationEigenVector.transpose() << "\n\n");
                }
        }

        this->totalContribution_.row(eigenvectorIndex + 3).col(0) << sumOfLowerAndHigherAlignmentRegion;
        this->highContribution_.row(eigenvectorIndex + 3).col(0) << sumOfHigherAlignmentRegion;

        // Cleanup
        sumOfLowerAndHigherAlignmentRegion = 0.0f;
        sumOfHigherAlignmentRegion = 0.0f;

        const Eigen::Matrix<T, 3, 1>& rotationEigenVector =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.template block<3, 1>(
                0, eigenvectorIndex);
        // Set flags (Rotation)
        if (detectLocalizability(
                sumOfLowerAndHigherAlignmentRegion, sumOfHigherAlignmentRegion, rotationAlignmentVector, rotationEigenVector))
        {
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(eigenvectorIndex).col(0)
                << static_cast<T>(LocalizabilityCategory::kLocalizable);
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_.rotationConstraintValues_
                    .row(eigenvectorIndex)
                    .col(0)
                << 1.0f;
            if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM("Sufficient Information: " << sumOfLowerAndHigherAlignmentRegion << " / "
                                                        << this->localizabilityDetectionParameters.enoughInformationThreshold << " and "
                                                        << " Upper: " << sumOfHigherAlignmentRegion << " / "
                                                        << this->localizabilityDetectionParameters.insufficientInformationThreshold << "\n"
                                                        << "Rotation Eigenvector Index " << eigenvectorIndex << " is "
                                                        << rotationEigenVector.transpose() << "\n\n");
            }
        }
        else
        {
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(eigenvectorIndex).col(0)
                << static_cast<T>(LocalizabilityCategory::kNonLocalizable);
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_.rotationConstraintValues_
                    .row(eigenvectorIndex)
                    .col(0)
                << 0.0f;
            if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_INFO_STREAM("Not Enough Information: " << sumOfLowerAndHigherAlignmentRegion << " < "
                                                        << this->localizabilityDetectionParameters.enoughInformationThreshold << " and "
                                                        << " Upper: " << sumOfHigherAlignmentRegion << " < "
                                                        << this->localizabilityDetectionParameters.insufficientInformationThreshold << "\n"
                                                        << "Rotation Eigenvector Index " << eigenvectorIndex << " is "
                                                        << rotationEigenVector.transpose() << "\n\n");
            }
        }

        // Plotting
        this->totalContribution_.row(eigenvectorIndex).col(0) << sumOfLowerAndHigherAlignmentRegion;
        this->highContribution_.row(eigenvectorIndex).col(0) << sumOfHigherAlignmentRegion;


        sumOfLowerAndHigherAlignmentRegion = 0.0f;
        sumOfHigherAlignmentRegion = 0.0f;
    }

    // Rotate the eigenvectors back to map frame.
    //(TODO) Can't we just multiply as matrix?
    for (Eigen::Index i = 0; i < 3; i++)
    {

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(i);


        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);

        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i) =
            this->localizabilityDetectionParameters.transformationToOptimizationFrame.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(i);
    }

    // Check if there is a degenerate direction. If there is no degenerate direction, then we won't do detection for the following iterations.
    // Set the repeatition to false, only re-activate if a direction is nonLocalizable.
    localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_ = false;
    for (Eigen::Index i = 0; i < 3; i++)
    {
        const T degeneracyLevelRPY =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(i).col(0).value();
        const T degeneracyLevelXYZ =
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(i).col(0).value();

        if ((degeneracyLevelXYZ == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
            || (degeneracyLevelRPY == static_cast<T>(LocalizabilityCategory::kNonLocalizable)))
        {
            localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_ = true;
            break;
        }
    }

    if (!localizabilityParametersForErrorMinimization.debugging_.whetherToRepeatEigenAnalysis_)
    {
        MELO_DEBUG_STREAM("All directions are localizable. Will not repeat localizability analysis.");
    }
    return true;
}

template<typename T>
bool PointMatcher<T>::ICP::detectLocalizabilityWithSolutionRemappingMethod(
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization, const T& eigenValuethreshold)
{

    if (localizabilityParametersForErrorMinimization.A_ == Eigen::Matrix<T, 6, 6>::Zero(6, 6))
    {
        MELO_DEBUG_STREAM("Optimization hessian is empty. This is not expected.");
        return false;
    }

    // State of the art localizability detection method uses 6 dimensional eigenvectors.
    Eigen::Matrix<T, 6, 6> sixDOFeigenvectors = Matrix::Zero(6, 6);
    Eigen::Matrix<T, 6, 1> sixDOFeigenvalues = Vector::Zero(6, 1);

    // 6x6 Eigenanalysis.
    eigenAnalysis(localizabilityParametersForErrorMinimization.A_, sixDOFeigenvectors, sixDOFeigenvalues);

    if ((sixDOFeigenvectors == Matrix::Zero(6, 6)) || (sixDOFeigenvalues == Vector::Zero(6, 1)))
    {
        MELO_DEBUG_STREAM("Eigenvector matrix or eigenvaleue vector is empty. This is not expected.");
        return false;
    }

    this->optimizationConditionNumber_ = sixDOFeigenvalues.maxCoeff() / sixDOFeigenvalues.minCoeff();
    this->optimizationEigenvalues = sixDOFeigenvalues;

    if (this->localizabilityDetectionParameters.useSolutionRemapping2019)
    {
        MELO_WARN_STREAM("Using Sol Remap 2019");
        solutionRemappingProjectionCalculation(
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.solutionRemappingProjectionMatrix_,
            sixDOFeigenvectors,
            sixDOFeigenvalues,
            this->optimizationConditionNumber_);
    }
    else
    {
        // Projection matrix calculation for solution remapping.
        solutionRemappingProjectionCalculation(
            localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.solutionRemappingProjectionMatrix_,
            sixDOFeigenvectors,
            sixDOFeigenvalues,
            eigenValuethreshold);
    }


    if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.solutionRemappingProjectionMatrix_
        == Matrix::Zero(6, 6))
    {
        MELO_DEBUG_STREAM("Solution Remapping projection matrix is all zeros. This is not expected.");
        return false;
    }

    return true;
}

template<typename T>
void PointMatcher<T>::ICP::solveSimpleOptimizationProblemForPartialConstraints(
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization, DataPoints& cloud,
    const bool isRotationSubspace, const Eigen::Index& index, const TransformationParameters& transformationToOptimizationFrame)
{
    // Got the cloud. Now Solve it.
    BOOST_AUTO(partialNormals, cloud.getDescriptorViewByName("matched_reference_normals"));
    // Critical note : Delta_features are not getting transformed on the backend like surface normals. This is intended but beware for future development.
    BOOST_AUTO(partial_deltas, cloud.getDescriptorViewByName("delta_features"));
    // dot product of dot = dot(deltas, normals)
    Matrix dotProd = Matrix::Zero(1, partialNormals.cols());
    Vector x_partial = Vector::Zero(3, 1);
    T thr = 0.00001f;
    if (!isRotationSubspace)
    {
        Matrix partial_A = partialNormals * partialNormals.transpose();

        // Calculate the dot product.
        for (int i = 0; i < partialNormals.rows(); i++)
        {
            dotProd += (partial_deltas.row(i).array() * partialNormals.row(i).array()).matrix();
        }


        // Partial LU pivotting (New method, more structured working)
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> new_A = l.transpose() * l;
        Vector new_b = l.transpose() * (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        x_partial = u.inverse() * y;

        /*
        // Partial LU pivotting  we wrote this works but in seemhule gave different result.
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> M = l * l.transpose();
        Eigen::Matrix<T, 3, 3> new_A = l * l.transpose()*l;
        Vector new_b = l * l.transpose() * (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        x_partial = u.inverse() * y;
        */

        //MELO_WARN_STREAM("Value " << new_A);


        /*
        // Partial LU pivotting (New method, more structured working)
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        //Eigen::Matrix<T, 3, 3> new_A = l.transpose()*l;

        Eigen::Matrix<T, 3, 3> M = l * l.transpose();
        Eigen::Matrix<T, 3, 3> new_A = partial_A * M.inverse();

        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(partial_b);
        x_partial = M.inverse() * y;
        */


        // Partial LU pivotting (L transpose removed still working)


        ////// Wrote that `this is working`
        /*
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        //Eigen::Matrix<T, 3, 3> new_A = l.transpose()*l;
        Eigen::Matrix<T, 3, 3> new_A = l;
        //Vector new_b = l.transpose() * (lu.permutationP() * partial_b);
        Vector new_b = (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        //x_partial = u.inverse() * y;
        // Second SVD
        x_partial = u.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);

        */


        //Eigen::SparseMatrix<T> M(3,3);
        //M = u.sparseView();
        //////
        //Eigen::BiCGSTAB<Eigen::SparseMatrix<T>, Eigen::IncompleteCholesky<T>> solver; //Eigen::IdentityPreconditioner Eigen::IncompleteLUT<T> Eigen::IncompleteCholesky<T> Eigen::DiagonalPreconditioner<T>
        //Eigen::ConjugateGradient<Eigen::SparseMatrix<T>, Eigen::Lower|Eigen::Upper,Eigen::IncompleteLUT<T > > solver;
        //solver.preconditioner().setDroptol(thr);
        //solver.compute(M);
        //x_partial=solver.solve(y);

        /*
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> new_A = l;
        Eigen::Matrix<T, 3, 3> new_A_ONLY_L = l;
        Vector new_b = (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        x_partial = u.inverse() * y;
        */


        // BACKUP
        // Partial LU pivotting
        /*
        const Vector partial_b = -(partialNormals * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> new_A = l.transpose()*l;
        Eigen::Matrix<T, 3, 3> new_A_ONLY_L = l;
        Vector new_b = l.transpose() * (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        x_partial = u.inverse() * y;
        */

        /*
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_no_precond(partial_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3 = svdCondNum_no_precond.singularValues();
        T condnum_no_precondition = eigenvalues3.maxCoeff()/eigenvalues3.minCoeff();
        
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_yes_precond(new_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3_yes = svdCondNum_yes_precond.singularValues();
        T condnum_yes_precondition = eigenvalues3_yes.maxCoeff()/eigenvalues3_yes.minCoeff();

        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_U_MAT(u, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3_U_MAT = svdCondNum_U_MAT.singularValues();
        T condnum_U_MAT = eigenvalues3_U_MAT.maxCoeff()/eigenvalues3_U_MAT.minCoeff();

        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("W/precond= " << condnum_yes_precondition << " WO/precond= " << condnum_no_precondition << " U-MAT= " << condnum_U_MAT);
        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        */

        // b = -(wF' * dot)
        //const Vector partial_b = -(partialNormals * dotProd.transpose());
        //x_partial = partial_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(partial_b);

        Vector rotatedEigenVector = localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(index);
        rotatedEigenVector = transformationToOptimizationFrame.topLeftCorner(3, 3) * rotatedEigenVector;


        const T constraintValue = rotatedEigenVector.transpose() * x_partial;
        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_.translationConstraintValues_
                .row(index)
                .col(0)
            << constraintValue;

        if(this->localizabilityDetectionParameters.isPrintingEnabled_){    
            MELO_WARN_STREAM("Partial constraint calc. Value " << constraintValue);
        }
    }
    else
    {
        const Matrix cross = this->errorMinimizer->crossProduct(cloud.features, partialNormals);
        Matrix partial_A = cross * cross.transpose();

        // Calculate the dot product.
        for (int i = 0; i < partialNormals.rows(); i++)
        {
            dotProd += (partial_deltas.row(i).array() * partialNormals.row(i).array()).matrix();
        }


        // L transpose removed


        /////   Wrote that this is working
        /*
        const Vector partial_b = -(cross * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> new_A = l;
        Vector new_b =  (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        //x_partial = u.inverse() * y;
        // Second SVD
        x_partial = u.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
        */


        //Eigen::SparseMatrix<T> M(3,3);
        //M = u.sparseView();
        //////
        //Eigen::BiCGSTAB<Eigen::SparseMatrix<T>, Eigen::IncompleteCholesky<T>> solver; //Eigen::IdentityPreconditioner Eigen::IncompleteLUT<T> Eigen::DiagonalPreconditioner<T>
        //Eigen::ConjugateGradient<Eigen::SparseMatrix<T>, Eigen::Lower|Eigen::Upper,Eigen::IncompleteLUT<T > > solver;
        //solver.preconditioner().setDroptol(thr);
        //solver.compute(M);
        //x_partial=solver.solve(y);

        /*
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_no_precond(partial_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3 = svdCondNum_no_precond.singularValues();
        T condnum_no_precondition = eigenvalues3.maxCoeff()/eigenvalues3.minCoeff();
        
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_yes_precond(new_A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3_yes = svdCondNum_yes_precond.singularValues();
        T condnum_yes_precondition = eigenvalues3_yes.maxCoeff()/eigenvalues3_yes.minCoeff();

        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svdCondNum_U_MAT(u, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<T, 3, 1> eigenvalues3_U_MAT = svdCondNum_U_MAT.singularValues();
        T condnum_U_MAT = eigenvalues3_U_MAT.maxCoeff()/eigenvalues3_U_MAT.minCoeff();

        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("++++++++++++++++++++++++++ROTATION++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("W/precond= " << condnum_yes_precondition << " WO/precond= " << condnum_no_precondition << " U-MAT= " << condnum_U_MAT);
        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        MELO_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        */

        // back up

        const Vector partial_b = -(cross * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        Eigen::Matrix<T, 3, 3> new_A = l.transpose() * l;
        Vector new_b = l.transpose() * (lu.permutationP() * partial_b);
        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(new_b);
        x_partial = u.inverse() * y;


        /*
        // Partial LU pivotting (New method, more structured working)
        const Vector partial_b = -(cross * dotProd.transpose());
        Vector y;
        Eigen::PartialPivLU<Eigen::Matrix<T, 3, 3>> lu(partial_A);
        Eigen::Matrix<T, 3, 3> l = Eigen::Matrix<T, 3, 3>::Identity();
        l.block(0, 0, 3, 3).template triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        Eigen::Matrix<T, 3, 3> u = lu.matrixLU().template triangularView<Eigen::Upper>();
        //Eigen::Matrix<T, 3, 3> new_A = l.transpose()*l;

        Eigen::Matrix<T, 3, 3> M = l * l.transpose();
        Eigen::Matrix<T, 3, 3> new_A = partial_A * M.inverse();

        y = new_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(partial_b);
        x_partial = M.inverse() * y;
        */

        // b = -(wF' * dot)
        //const Vector partial_b = -(cross * dotProd.transpose());
        //x_partial = partial_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(partial_b);


        // ENABLE IF SUDDEN CHANGE
        //x_partial = Eigen::Matrix<T, 3, 1>::Zero(3, 1);

        // Rotate the eigenvector back to optimization frame.
        Vector rotatedEigenVector = localizabilityParametersForErrorMinimization.T_convert_from_base_to_lidar_mat.topLeftCorner(3, 3)
            * localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(index);
        rotatedEigenVector = transformationToOptimizationFrame.topLeftCorner(3, 3) * rotatedEigenVector;

        const T constraintValue = rotatedEigenVector.transpose() * x_partial;
        localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_.rotationConstraintValues_
                .row(index)
                .col(0)
            << constraintValue;
        if(this->localizabilityDetectionParameters.isPrintingEnabled_){
            MELO_WARN_STREAM("Partial constraint calc. Value " << constraintValue);
        }
    }
}

template<typename T>
bool PointMatcher<T>::ICP::compareAlignmentList(const std::pair<Eigen::Index, T>& p1, const std::pair<Eigen::Index, T>& p2)
{
    // std::make_heap and std::sort_heap expects first element to be smaller than the second one to return true.
    // Other wise we would use p1.second > p2.second. E.g. to to use std::sort.
    // We implement for std::partial_sort
    return p1.second > p2.second;
}

template<typename T>
Eigen::Matrix<T, 3, 6> PointMatcher<T>::ICP::registerDegenerateDirections(const LocalizabilityParametersForErrorMinimization& params) const
{
    Eigen::Matrix<T, 3, 6> degenerateDirection = Eigen::Matrix<T, 3, 6>::Zero(3, 6);

    // Go over the values.
    for (Eigen::Index j = 0; j < params.localizabilityAnalysisResults_.localizabilityXyz_.rows(); j++)
    {
        if (params.localizabilityAnalysisResults_.localizabilityRpy_.row(j).value()
            == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
        {
            degenerateDirection.col(j) = params.localizabilityAnalysisResults_.rotationEigenvectors_.col(j);
        }

        if (params.localizabilityAnalysisResults_.localizabilityXyz_.row(j).value()
            == static_cast<T>(LocalizabilityCategory::kNonLocalizable))
        {
            degenerateDirection.col(j + 3) = params.localizabilityAnalysisResults_.translationEigenvectors_.col(j);
        }
    }

    return (degenerateDirection);
}

template struct PointMatcher<float>::ICPSequence;
template struct PointMatcher<double>::ICPSequence;
