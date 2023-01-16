from opensfm import dataset
from opensfm import dense
from opensfm.dataset import DataSet
from opensfm.dataset import DataSet
import open3d as o3d
from opensfm.geo import (ecef_from_lla, topocentric_from_lla)
from opensfm import (
    io,
)
import os
import numpy as np

# import cv2


def generate_gps_data():
    data = DataSet('data/odm_data_aukerman')
    subfolder = 'undistorted'
    interactive = False

    undistorted_data_path = os.path.join(data.data_path, subfolder)
    undistorted_dataset = dataset.UndistortedDataSet(
        data, undistorted_data_path, io_handler=data.io_handler)
    data.config["interactive"] = interactive

    reconstructions = undistorted_dataset.load_undistorted_reconstruction()
    tracks_manager = undistorted_dataset.load_undistorted_tracks_manager()
    dense.compute_depthmaps(undistorted_dataset,
                            tracks_manager, reconstructions[0])

    images = tracks_manager.get_shot_ids()
    data.init_reference(images)

    # A list that is used to store the distances of point from the line
    # defined by the origin of the camera and the point on the image defined
    # by the pixel coordinates (x, y).
    distances = []
    for reconstruction in reconstructions:
        point_cloud = io.reconstruction_to_ply(
            reconstruction, tracks_manager, False, False, False)

        file = open(data.data_path + '/reconstruction.ply', 'a')
        file.write(point_cloud)
        file.close()

        point_cloud = o3d.io.read_point_cloud(
            data.data_path + '/reconstruction.ply')

        image_x_pixel = 1643
        image_y_pixel = 2123
        distances = []
        results = {}
        image = images[0]

        # image_path = data.data_path + '/images/' + image
        # print('image_path:', image_path)
        # image_path = image_path.replace('.JPG.jpg', '.JPG')
        # print('image_path:', image_path)
        # img = cv2.imread(image_path)
        # cv2.imshow('image', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        shot = reconstruction.shots[image]
        pose = shot.pose
        cam = shot.camera

        p1 = shot.pose.get_origin()
        p2 = cam.pixel_to_normalized_coordinates(
            [image_x_pixel, image_y_pixel])

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

        distances.append(closest_point_in_cloud)

        results['distances'] = distances
        results['shot_id'] = shot.id
        results['closest_point_in_cloud'] = closest_point_in_cloud
        gps_world = reconstruction.reference.to_lla(*closest_point_in_cloud)

        results['closest_point_in_cloud_lla'] = gps_world
        results['image_x_pixel'] = image_x_pixel
        results['image_y_pixel'] = image_y_pixel

        r = shot.pose.get_rotation_matrix().T
        p2 = shot.pose.inverse().transform(bearing)
        pos_bearing_scale = r.dot(p2)
        results['pos_bearing_scale'] = pos_bearing_scale

        gps_world = reconstruction.reference.to_lla(*pos_bearing_scale)
        results['pos_bearing_scale_lla'] = gps_world

        p2 = cam.pixel_to_normalized_coordinates(
            [image_x_pixel, image_y_pixel])
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

        print('results:', results)


generate_gps_data()
