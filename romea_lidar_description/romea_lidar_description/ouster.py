from . import common


def get_model_family(model):
    if 'os1' in model:
        return 'os1'

    raise RuntimeError(
        f'Ouster {model} lidar is not supported by romea_lidar_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_os1_complete_configuration(model, rate, resolution):
    specifications = common.get_lidar_specifications('ouster', 'os1')

    res_index = -1
    for i, available_res in enumerate(specifications['azimut_angle_increment']):
        if abs(resolution - available_res) < 1e-5:
            res_index = i
            break

    if res_index == -1:
        raise ValueError(f'Resolution {resolution} is not available for Ouster OS1 lidar')

    if rate not in specifications['rate']:
        raise ValueError(f'Rate {rate} is not available for Ouster OS1 lidar')

    return {
        'type': specifications['type'],
        'minimal_azimut_angle': specifications['minimal_azimut_angle'],
        'maximal_azimut_angle': specifications['maximal_azimut_angle'],
        'azimut_angle_increment': specifications['azimut_angle_increment'][res_index],
        'azimut_angle_std': specifications['azimut_angle_std'],
        'samples': specifications['samples'][res_index],
        'minimal_elevation_angle': specifications['minimal_elevation_angle'],
        'maximal_elevation_angle': specifications['maximal_elevation_angle'],
        'elevation_angle_increment': specifications['elevation_angle_increment'][model],
        'elevation_angle_std': specifications['elevation_angle_std'],
        'lasers': specifications['lasers'][model],
        'minimal_range': specifications['minimal_range'],
        'maximal_range': specifications['maximal_range'],
        'range_std': specifications['range_std'],
        'rate': rate,
    }


def get_lidar_complete_configuration(model, rate, resolution):

    family = get_model_family(model)

    if family == 'os1':
        return get_os1_complete_configuration(model, rate, resolution)


def get_lidar_geometry_file_path(model):
    return common.get_lidar_geometry_file_path("sick", get_model_family(model))


def get_lidar_geometry(model):
    return common.get_lidar_geometry("sick", get_model_family(model))
