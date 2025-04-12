local base_params = include("default/default_anymal_parameters.lua")
params = deepcopy(base_params)

-- MAPPER_LOCALIZER
params.mapper_localizer.is_carving_enabled = true
params.mapper_localizer.min_movement_between_mapping_steps = 2.0
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.1
params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 2.0 --sec

-- SUBMAP
params.submap.submap_size = 20.0 --meters
params.submap.submaps_num_scan_overlap = 1

-- MAP_BUILDER
params.map_builder.map_voxel_size = 0.1

return params