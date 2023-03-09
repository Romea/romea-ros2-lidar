#!/usr/bin/env python3

# import xacro
# import yaml

# from ament_index_python.packages import get_package_share_directory


def test_lidar_description():
    # print("coucou")
    assert False


if __name__ == "__main__":
    test_lidar_description()

# def sick_lms1xx_configuration(model, rate, resolution):

#    specifications_file = (
#        get_package_share_directory("romea_lidar_description")
#        + "/config/sick_lms1xx_specifications.yaml"
#    )

#    with open(specifications_file) as f:
#        specifications = yaml.safe_load(f)

#    if not rate and not resolution:
#        raise ValueError("Rate or resolution must be defined in lidar configuration")

#    if rate is not None:

#        if rate in specifications["rate"]:
#            rate_hz = str(rate) + "hz"
#        else:
#            raise ValueError("Rate " + str(rate) + " is not available for lms1xx lidar")

#        if resolution is not None:

#            if resolution != specifications["azimut_angle_increment"][rate_hz]:
#                raise ValueError(
#                    "Rate "
#                    + str(rate)
#                    + " + resolution "
#                    + str(resolution)
#                    + " configuration is not available for lms1xx lidar"
#                )

#    else:

#        if resolution in specifications["azimut_angle_increment"].values():
#            rate = dict.keys()[dict.values().index(resolution)]
#            rate_hz = str(rate) + "hz"
#        else:
#            raise ValueError(
#                "Resolution " + str(resolution) + " is not available for lms1xx lidar"
#            )
#    sub_model = model[0:5] + "x"

#    return {
#        "minimal_azimut_angle": specifications["minimal_azimut_angle"],
#        "maximal_azimut_angle": specifications["maximal_azimut_angle"],
#        "azimut_angle_increment": specifications["azimut_angle_increment"][rate_hz],
#        "azimut_angle_std": specifications["azimut_angle_std"],
#        "minimal_range": specifications["minimal_range"],
#        "maximal_range": specifications["maximal_range"][sub_model],
#        "range_std": specifications["range_std"],
#        "samples": specifications["samples"][rate_hz],
#        "rate": rate
#    }

# def sick_tim5xx_configuration(model, rate, resolution):

#    specifications_file = (
#        get_package_share_directory("romea_lidar_description")
#        + "/config/sick_lms1xx_specifications.yaml"
#    )

#    with open(specifications_file) as f:
#        specifications = yaml.safe_load(f)

#    if rate is not None and rate != specifications["rate"]:
#        raise ValueError("Rate " + str(rate) + " is not available for tim5xx lidar")

#    if resolution is not None and resolution!=specifications["azimut_angle_increment"][model]:
#        raise ValueError(
#            "Resolution " + str(resolution) + " is not available for tim5xx lidar"
#        )

#    return {
#        "minimal_azimut_angle": specifications["minimal_azimut_angle"],
#        "maximal_azimut_angle": specifications["maximal_azimut_angle"],
#        "azimut_angle_increment": specifications["azimut_angle_increment"][model],
#        "azimut_angle_std": specifications["azimut_angle_std"],
#        "minimal_range": specifications["minimal_range"],
#        "maximal_range": specifications["maximal_range"][model],
#        "range_std": specifications["range_std"],
#        "samples": specifications["samples"][model],
#        "rate": rate
#    }


# def urdf(prefix, name, type, model, rate, resolution, parent_link, xyz, rpy):

#    if "lms1" in model:
#        configuration=sick_lms1xx_configuration(model, rate, resolution)

#        xacro_file = (
#            get_package_share_directory("romea_lidar_description")
#            + "/urdf/sick_lms1xx.xacro.urdf"
#        )

#        model = model[0:5] + "x"


#    if "tim5" in model:
#        configuration=sick_tim5xx_configuration(model, rate, resolution)

#        xacro_file = (
#            get_package_share_directory("romea_lidar_description")
#            + "/urdf/sick_lms5xx.xacro.urdf"
#        )

#    urdf_xml = xacro.process_file(
#        xacro_file,
#        mappings={
#            "prefix": prefix,
#            "name": name,
#            "model": model,
#            "parent_link": parent_link,
#            "xyz": " ".join(map(str, xyz)),
#            "rpy": " ".join(map(str, rpy)),
#            "mesh_visual": str(True),
#        },
#    )

#    return urdf_xml.toprettyxml()
