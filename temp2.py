from opensfm import dataset
from opensfm import dense
from opensfm.dataset import DataSet
from opensfm.dataset import DataSet
from opensfm.types import Reconstruction
import open3d as o3d
from opensfm.geo import (ecef_from_lla, topocentric_from_lla)
from opensfm import (
    io,
)
import os
import numpy as np
from typing import Union
import xml.etree.ElementTree as ET

import pdb
# import cv2


def generate_gps_data(
    dataset_name: str,
    image_file: Union[str, None] = None,
    image_x_pixel: Union[int, None] = None,
    image_y_pixel: Union[int, None] = None,
):

    data = DataSet('data/' + dataset_name)

    undistorted_data_path = os.path.join(data.data_path, 'undistorted')
    undistorted_dataset = dataset.UndistortedDataSet(
        data,
        undistorted_data_path,
        io_handler=data.io_handler,
    )
    data.config["interactive"] = False

    reconstructions = undistorted_dataset.load_undistorted_reconstruction()
    tracks_manager = undistorted_dataset.load_undistorted_tracks_manager()
    dense.compute_depthmaps(
        undistorted_dataset,
        tracks_manager,
        reconstructions[0],
    )

    images = tracks_manager.get_shot_ids()

    data.init_reference(images)

    # A list that is used to store the distances of point from the line
    # defined by the origin of the camera and the point on the image defined
    # by the pixel coordinates (x, y).
    distances = []
    for reconstruction in reconstructions:
        point_cloud = io.reconstruction_to_ply(
            reconstruction,
            tracks_manager,
            False,
            False,
            False,
        )

        file = open(data.data_path + '/reconstruction.ply', 'a')
        file.write(point_cloud)
        file.close()

        point_cloud = o3d.io.read_point_cloud(
            data.data_path + '/reconstruction.ply')

        if image_file != None:
            images = [image_file]

        results_for_mission = {}

        for image in images:
            pixel_coordinates = [(image_x_pixel, image_y_pixel)]
            if image_x_pixel == None and image_y_pixel == None:
                pixel_coordinates = [(i, i) for i in range(0, 3000, 5)]

            results_for_image = []

            for pixel_coordinate in pixel_coordinates:
                results_for_pixel = calculate_coordinates(
                    reconstruction,
                    point_cloud,
                    image,
                    pixel_coordinate[0],
                    pixel_coordinate[1],
                )
                results_for_image.append(results_for_pixel)

            results_for_mission[image] = results_for_image

        create_kml(data.data_path + "/coordinates.kml", results_for_mission)


def calculate_coordinates(
    reconstruction: Reconstruction,
    point_cloud: any,
    image: str,
    x_pixel: int,
    y_pixel: int,
):
    results = {}

    shot = reconstruction.shots[image]
    pose = shot.pose
    cam = shot.camera

    p1 = shot.pose.get_origin()
    p2 = cam.pixel_to_normalized_coordinates(
        [x_pixel, y_pixel]
    )

    bearing = cam.pixel_bearing(p2)
    scale = 1 / bearing[2]
    bearing = scale * bearing

    p2 = pose.inverse().transform(bearing)
    p2_lla = reconstruction.reference.to_lla(*p2)
    results['p2_lla'] = p2_lla

    points = np.asarray(point_cloud.points)  # point cloud

    res = np.linalg.norm(
        np.cross(p2 - p1, p1 - points),
        axis=1
    ) / np.linalg.norm(p2 - p1)
    # print("np.cross",np.cross(p2-p1, p1-points))
    # print("np.linalg.norm",np.linalg.norm(np.cross(p2-p1, p1-points), axis=1))
    # print("np.linalg.norm(p2-p1)",np.linalg.norm(p2-p1))
    # print("res",res)

    results['shot_id'] = shot.id

    # closest_point_in_cloud is storing the closest point in the point cloud
    # to the line defined by the origin of the camera and the point on the
    # image defined by the pixel coordinates (x, y).
    closest_point_in_cloud = point_cloud.points[np.argmin(res)]
    results['closest_point_in_cloud'] = closest_point_in_cloud

    closest_point_in_cloud_lla = reconstruction.reference.to_lla(
        *closest_point_in_cloud,
    )
    results['closest_point_in_cloud_lla'] = closest_point_in_cloud_lla

    results['x_pixel'] = x_pixel
    results['y_pixel'] = y_pixel

    r = shot.pose.get_rotation_matrix().T
    p2 = shot.pose.inverse().transform(bearing)
    pos_bearing_scale = r.dot(p2)
    results['pos_bearing_scale'] = pos_bearing_scale

    pos_bearing_scale_lla = reconstruction.reference.to_lla(*pos_bearing_scale)
    results['pos_bearing_scale_lla'] = pos_bearing_scale_lla

    p2 = cam.pixel_to_normalized_coordinates(
        [x_pixel, y_pixel])
    bearing = cam.pixel_bearing(p2)
    r = shot.pose.get_rotation_matrix().T
    pos_bearing = r.dot(bearing)
    results['pos_bearing'] = pos_bearing

    pos_bearing_lla = reconstruction.reference.to_lla(*pos_bearing)
    results['pos_bearing_lla'] = pos_bearing_lla

    p1 = shot.pose.get_origin()
    pos_origin = r.dot(p1)
    results['pos_origin'] = pos_origin

    pos_origin_lla = reconstruction.reference.to_lla(*pos_origin)
    results['pos_origin_lla'] = pos_origin_lla

    return results


def create_kml(path: str, results_for_mission: dict):
    kml = ET.Element("kml")
    document = ET.SubElement(kml, "Document")

    for image in results_for_mission:
        for results_for_image in results_for_mission[image]:
            for results_for_pixel in results_for_image:
                coordinates = results_for_pixel['closest_point_in_cloud_lla']
                placemark = ET.SubElement(document, "Placemark")
                text = image.replace('.jpg', '') + '_' \
                    + results_for_pixel['x_pixel'] + '_' \
                    + results_for_pixel['y_pixel']
                ET.SubElement(placemark, "name").text = text
                point = ET.SubElement(placemark, "Point")
                ET.SubElement(
                    point,
                    "coordinates",
                ).text = f"{coordinates[1]},{coordinates[0]},{coordinates[2]}"

    pdb.set_trace()

    tree = ET.ElementTree(kml)
    tree.write(
        path,
        xml_declaration=True,
        encoding='utf-8',
        method="xml",
    )


generate_gps_data(
    'odm_data_aukerman',
    # 'DSC00232.JPG',
)
