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
# from xml.dom.minidom import Document
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

        distances = []

        # pdb.set_trace()
        if image_file != '':
            images = [image_file]

        for image in images:
            pixel_coordinates = [(image_x_pixel, image_y_pixel)]
            if image_x_pixel == None and image_y_pixel == None:
                coordinates = [(i, i) for i in range(0, 3000, 5)]

            print('pixel_coordinates:', pixel_coordinates)

            all_results = []
            gps_coordinates = []

            for pixel_coordinate in pixel_coordinates:
                results = calculate_coordinates(
                    reconstruction,
                    point_cloud,
                    image,
                    pixel_coordinate[0],
                    pixel_coordinate[1],
                )
                all_results.append(results)

                distances.append(results['closest_point_in_cloud'])
                gps_coordinates.append(results['closest_point_in_cloud_lla'])
                pdb.set_trace()
                # print('distances:', distances)

            # google_maps_url = create_google_map_url(all_results)
            # print('google maps url:', google_maps_url)

            create_kml(data.data_path + "/coordinates.kml", gps_coordinates)


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
        [x_pixel, y_pixel])

    bearing = cam.pixel_bearing(p2)
    scale = 1 / bearing[2]
    bearing = scale * bearing

    p2 = pose.inverse().transform(bearing)

    gps_world = reconstruction.reference.to_lla(*p2)

    results['p2_lla'] = gps_world
    points = np.asarray(point_cloud.points)  # point cloud

    res = np.linalg.norm(
        np.cross(p2 - p1, p1 - points), axis=1) / np.linalg.norm(p2 - p1)
    # print("np.cross",np.cross(p2-p1, p1-points))
    # print("np.linalg.norm",np.linalg.norm(np.cross(p2-p1, p1-points), axis=1))
    # print("np.linalg.norm(p2-p1)",np.linalg.norm(p2-p1))
    # print("res",res)

    # closest_point_in_cloud is storing the closest point in the point cloud
    # to the line defined by the origin of the camera and the point on the
    # image defined by the pixel coordinates (x, y).
    closest_point_in_cloud = point_cloud.points[np.argmin(res)]

    results['shot_id'] = shot.id
    results['closest_point_in_cloud'] = closest_point_in_cloud
    gps_world = reconstruction.reference.to_lla(*closest_point_in_cloud)

    results['closest_point_in_cloud_lla'] = gps_world
    results['x_pixel'] = x_pixel
    results['y_pixel'] = y_pixel

    r = shot.pose.get_rotation_matrix().T
    p2 = shot.pose.inverse().transform(bearing)
    pos_bearing_scale = r.dot(p2)
    results['pos_bearing_scale'] = pos_bearing_scale

    gps_world = reconstruction.reference.to_lla(*pos_bearing_scale)
    results['pos_bearing_scale_lla'] = gps_world

    p2 = cam.pixel_to_normalized_coordinates(
        [x_pixel, y_pixel])
    bearing = cam.pixel_bearing(p2)
    r = shot.pose.get_rotation_matrix().T
    pos_bearing = r.dot(bearing)
    results['pos_bearing'] = pos_bearing

    gps_world = reconstruction.reference.to_lla(*pos_bearing)
    results['pos_bearing_lla'] = gps_world

    p1 = shot.pose.get_origin()
    pos_origin = r.dot(p1)
    results['pos_origin'] = pos_origin

    gps_world = reconstruction.reference.to_lla(*pos_origin)
    results['pos_origin_lla'] = gps_world

    return results


def create_google_map_url(results: list):
    coordinates = []
    for result in results:
        coordinates.append(
            [
                result['closest_point_in_cloud_lla'][0],
                result['closest_point_in_cloud_lla'][1],
                0,
            ]
        )

    waypoints = "|".join(
        [f"{lat},{long},{alt}" for lat, long, alt in coordinates]
    )

    url = f"https://www.google.com/maps/dir/?api=1&waypoints={waypoints}"
    return url


def create_kml(path: str, gps_coordinates: list):
    kml = ET.Element("kml")
    document = ET.SubElement(kml, "Document")
    for coord in gps_coordinates:
        placemark = ET.SubElement(document, "Placemark")
        ET.SubElement(placemark, "name").text = "Coordinate"
        point = ET.SubElement(placemark, "Point")
        ET.SubElement(
            point, "coordinates").text = f"{coord[1]},{coord[0]},{coord[2]}"

    tree = ET.ElementTree(kml)
    tree.write(
        path,
        xml_declaration=True,
        encoding='utf-8',
        method="xml",
    )


generate_gps_data(
    'odm_data_aukerman',
    'DSC00232.JPG.jpg',
)
