local base_params = include("default/default_anymal_parameters.lua")
params = deepcopy(base_params)

--MAPPER_LOCALIZER
params.mapper_localizer.is_use_map_initialization = false
params.mapper_localizer.is_merge_scans_into_map = false
params.mapper_localizer.is_build_dense_map = false
params.mapper_localizer.is_attempt_loop_closures = false
params.mapper_localizer.is_print_timing_information = false
params.mapper_localizer.min_movement_between_mapping_steps = 0.0

params.mapper_localizer.is_carving_enabled = false
params.map_builder.space_carving.carve_space_every_n_scans = 10

params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.15
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 1.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 80.0 --We don't want to crop the scans.
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.2

params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 0.1 --sec

--SUBMAP
params.submap.submap_size = 40.0 --meters
params.submap.adjacency_based_revisiting_min_fitness = 0.5
params.submap.min_seconds_between_feature_computation = 5.0
params.submap.max_num_points = 7500000
params.submap.submaps_num_scan_overlap = 10

--MAP_BUILDER
params.map_builder.map_voxel_size = 0.2
params.map_builder.scan_cropping.cropping_radius_max = 80.0
params.map_builder.scan_cropping.cropping_radius_min = 0.2

return params