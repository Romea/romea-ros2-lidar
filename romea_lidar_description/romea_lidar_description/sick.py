from . import common


def get_model_family(model):
    if 'lms1' in model:
        return 'lms1xx'
    elif 'tim5' in model:
        return 'tim5xx'
    elif 'mrs1' in model:
        return 'mrs1xxx'

    raise RuntimeError(
        f'Sick {model} lidar is not supported by romea_lidar_description package. '
        'Please check your configuration or contribute to support this sensor.'
    )


def get_lms1xx_complete_configuration(model, rate, resolution):
    specifications = common.get_lidar_specifications("sick", "lms1xx")

    if not rate and not resolution:
        raise ValueError('Rate or resolution must be defined in lidar configuration')

    if rate is not None:
        if rate in specifications['rate']:
            rate_hz = str(rate) + 'hz'
        else:
            raise ValueError('A ' + str(rate) + 'Hz rate is not available for ' + model + ' lidar')

        if resolution is not None:
            if resolution != specifications['azimut_angle_increment'][rate_hz]:
                raise ValueError(
                    'A '
                    + str(rate)
                    + 'Hz rate and a '
                    + str(resolution)
                    + '° resolution is not an available configuration for '
                    + model
                    + ' lidar'
                )

    else:
        if resolution in specifications['azimut_angle_increment'].values():
            rate = dict.keys()[dict.values().index(resolution)]
            rate_hz = str(rate) + 'hz'
        else:
            raise ValueError(
                'A '
                + str(resolution) 
                + '° resolution is not available for '
                + model
                + ' lidar'
            )

    sub_model = model[0:5] + 'x'

    return {
        'type': specifications['type'],
        'minimal_azimut_angle': specifications['minimal_azimut_angle'],
        'maximal_azimut_angle': specifications['maximal_azimut_angle'],
        'azimut_angle_increment': specifications['azimut_angle_increment'][rate_hz],
        'azimut_angle_std': specifications['azimut_angle_std'],
        'minimal_range': specifications['minimal_range'],
        'maximal_range': specifications['maximal_range'][sub_model],
        'range_std': specifications['range_std'],
        'samples': specifications['samples'][rate_hz],
        'rate': rate,
    }


def get_tim5xx_complete_configuration(model, rate, resolution):
    specifications = common.get_lidar_specifications("sick", "tim5xx")

    if rate is not None and rate != specifications['rate']:
        raise ValueError(
            'A '
            + str(rate)
            + 'Hz rate is not available for '
            + model 
            + ' lidar'
        )

    if resolution is not None and resolution != specifications['azimut_angle_increment'][model]:
        raise ValueError(
            'A '
            + str(resolution)
            + '° resolution is not available for '
            + model 
            + ' lidar'
        )

    return {
        'type': specifications['type'],
        'minimal_azimut_angle': specifications['minimal_azimut_angle'],
        'maximal_azimut_angle': specifications['maximal_azimut_angle'],
        'azimut_angle_increment': specifications['azimut_angle_increment'][model],
        'azimut_angle_std': specifications['azimut_angle_std'],
        'minimal_range': specifications['minimal_range'],
        'maximal_range': specifications['maximal_range'][model],
        'range_std': specifications['range_std'],
        'samples': specifications['samples'][model],
        'rate': rate,
    }


def get_mrs1xxx_complete_configuration(rate, resolution):
    specifications = common.get_lidar_specifications("sick", "mrs1xxx")

    if rate is not None and rate != specifications['rate']:
        raise ValueError('Rate ' + str(rate) + ' is not available for mrs1xxx lidar')

    if resolution is not None and resolution not in specifications['azimut_angle_increment']:
        raise ValueError('Resolution ' + str(resolution) + ' is not available for mrs1xxx lidar')

    index = specifications['azimut_angle_increment'].index(resolution)

    return {
        'type': specifications['type'],
        'minimal_azimut_angle': specifications['minimal_azimut_angle'],
        'maximal_azimut_angle': specifications['maximal_azimut_angle'],
        'azimut_angle_increment': specifications['azimut_angle_increment'][index],
        'azimut_angle_std': specifications['azimut_angle_std'],
        'samples': specifications['samples'][index],
        'minimal_elevation_angle': specifications['minimal_elevation_angle'],
        'maximal_elevation_angle': specifications['maximal_elevation_angle'],
        'elevation_angle_increment': specifications['elevation_angle_increment'],
        'elevation_angle_std': specifications['elevation_angle_std'],
        'lasers': specifications['lasers'],
        'minimal_range': specifications['minimal_range'],
        'maximal_range': specifications['maximal_range'],
        'range_std': specifications['range_std'],
        'rate': rate,
    }




def get_lidar_complete_configuration(model, rate, resolution):

    family = get_model_family(model)

    if family == 'lms1xx':
        return get_lms1xx_complete_configuration(model, rate, resolution)
    elif family == 'tim5xx':
        return get_tim5xx_complete_configuration(model, rate, resolution)
    elif family == 'mrs1xxx':
        return get_mrs1xxx_complete_configuration(rate, resolution)


def get_lidar_specifications(model):
    return common.get_lidar_specifications("sick", get_model_family(model))


def get_lidar_specifications_file_path(model):
    return common.get_lidar_specifications_file_path("sick", get_model_family(model))


def get_lidar_geometry_file_path(model):
    return common.get_lidar_geometry_file_path("sick", get_model_family(model))


def get_lidar_geometry(model):
    return common.get_lidar_geometry("sick", get_model_family(model))
