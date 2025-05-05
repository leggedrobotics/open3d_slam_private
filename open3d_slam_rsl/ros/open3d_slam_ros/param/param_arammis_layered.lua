local base_params = include("default/default_anymal_parameters.lua")
params = deepcopy(base_params)

--MAPPER_LOCALIZER
params.mapper_localizer.is_use_map_initialization = false
params.mapper_localizer.is_merge_scans_into_map = false
params.mapper_localizer.is_build_dense_map = false
params.mapper_localizer.is_attempt_loop_closures = false
params.mapper_localizer.is_print_timing_information = false
params.mapper_localizer.min_movement_between_mapping_steps = 0.1

params.mapper_localizer.is_carving_enabled = true
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.05
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 1.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 60.0 --We don't want to crop the scans.
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.5
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 2.0 --NOT USED RIGHT NOW
params.mapper_localizer.scan_to_map_registration.icp.knn = 10 --Currently only used for surface normal estimation.
params.mapper_localizer.scan_to_map_registration.icp.max_distance_knn = 2.0 --Currently only used for surface normal estimation.
params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 2.0 --sec

--SUBMAP
params.submap.submap_size = 30.0 --meters
params.submap.adjacency_based_revisiting_min_fitness = 0.4
params.submap.min_seconds_between_feature_computation = 5.0
params.submap.max_num_points = 7500000
params.submap.submaps_num_scan_overlap = 20

--MAP_BUILDER
params.map_builder.map_voxel_size = 0.2
params.map_builder.scan_cropping.cropping_radius_max = 60.0
params.map_builder.scan_cropping.cropping_radius_min = 0.5
params.map_builder.space_carving.carve_space_every_n_scans = 1

return params