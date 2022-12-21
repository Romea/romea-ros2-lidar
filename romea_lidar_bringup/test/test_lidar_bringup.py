
import os
import pytest

from romea_lidar_bringup import LIDARMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(os.getcwd(),"test_lidar_bringup.yaml")
    return LIDARMetaDescription(meta_description_filename)


def test_get_name(meta_description):
    assert meta_description.get_name() == "lidar"

def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration()  == True

def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg()  == "sick_scan"

def test_get_driver_ip(meta_description):
    assert meta_description.get_driver_ip() == "192.168.1.112"

def test_get_driver_baudrate(meta_description):
    assert meta_description.get_driver_port() == 2112

def test_get_type(meta_description):
    assert meta_description.get_type() == "sick"

def test_get_model(meta_description):
    assert meta_description.get_model()  == "lms151"

def test_get_rate(meta_description):
    assert meta_description.get_rate() == 50

def test_get_resolution(meta_description):
    assert meta_description.get_resolution() == 0.5

def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"

def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]

def test_get_xyz(meta_description):
    assert meta_description.get_rpy() == [4.0, 5.0, 6.0]
