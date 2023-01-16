from opensfm import dataset
from opensfm import dense
from opensfm.dataset import DataSet
# from opensfm.commands import command
from opensfm.dataset import DataSet
import open3d as o3d
from opensfm.geo import (ecef_from_lla, topocentric_from_lla)
from opensfm import (
    io,
    tracking,
)
import os
import numpy as np

import pdb


def generate_gps_data():
    data = DataSet('data/odm_data_aukerman')
    subfolder = 'undistorted'
    interactive = False

    udata_path = os.path.join(data.data_path, subfolder)
    udataset = dataset.UndistortedDataSet(
        data, udata_path, io_handler=data.io_handler)
    data.config["interactive"] = interactive

    reconstructions = udataset.load_undistorted_reconstruction()
    tracks_manager = udataset.load_undistorted_tracks_manager()
    dense.compute_depthmaps(udataset, tracks_manager, reconstructions[0])

    report = {}
    config = data.config
    images = tracks_manager.get_shot_ids()
    data.init_reference(images)

    remaining_images = set(images)
    gcps = data.load_ground_control_points()
    print("path", udata_path)
    common_tracks = tracking.all_common_tracks_with_features(
        tracks_manager)
    distancias = []
    diff = []
    for reconstruction in reconstructions:

        nube = io.reconstruction_to_ply(
            reconstruction, tracks_manager, False, False, False)
        print("nube", nube)
        pdb.set_trace()

        f = open(data.data_path + '/reconstruction.ply', 'a')
        f.write(nube)
        f.close()

        nube = o3d.io.read_point_cloud(
            data.data_path + '/reconstruction.ply')

        x = 1024
        print("x", x)
        y = 1024
        print("y", y)
        distancias = []
        anterior = []
        diferenciasgps = []
        diferenciasunidades = []
        contador = 0
        valores = []
        for i in images:
            for b in range(0, 3000):
                shot = reconstruction.shots[i]
                print("shot", shot.id)
                pose = shot.pose
                cam = shot.camera
                x = b
                y = b
                print("x", x)
                print("y", y)
                p1 = shot.pose.get_origin()
                print("p1", p1)
                p2 = cam.pixel_to_normalized_coordinates([x, y])
                print("p2", p2)
                bearing = cam.pixel_bearing(p2)
                print("bearing", bearing)
                scale = 1 / bearing[2]
                print("scale", scale)
                bearing = scale * bearing
                print("bearing scale", bearing)
                p2 = pose.inverse().transform(bearing)
                print("p2 inverse", p2)
                gps_world = reconstruction.reference.to_lla(*p2)
                print("gps_world", gps_world)
                valores.append(gps_world)
                points = np.asarray(nube.points)  # point cloud
                print("points", points)
                res = np.linalg.norm(
                    np.cross(p2 - p1, p1 - points), axis=1) / np.linalg.norm(p2 - p1)
                # print("np.cross",np.cross(p2-p1, p1-points))
                # print("np.linalg.norm",np.linalg.norm(np.cross(p2-p1, p1-points), axis=1))
                # print("np.linalg.norm(p2-p1)",np.linalg.norm(p2-p1))
                # print("res",res)
                a = nube.points[np.argmin(res)]
                print("pos inter", a)
                distancias.append(a)
                valores.append(shot.id)
                valores.append(a)
                gps_world = reconstruction.reference.to_lla(*a)
                valores.append(gps_world)
                print("gps_world inter", gps_world)
                valores.append(x)
                valores.append(y)
                r = shot.pose.get_rotation_matrix().T
                p2 = shot.pose.inverse().transform(bearing)
                f = r.dot(p2)
                print("pos bearing escala", f)
                valores.append(f)
                gps_world = reconstruction.reference.to_lla(*f)
                valores.append(gps_world)
                print('\nGPS Worldyo bearing escala', gps_world)
                p2 = cam.pixel_to_normalized_coordinates([x, y])
                bearing = cam.pixel_bearing(p2)
                r = shot.pose.get_rotation_matrix().T
                f = r.dot(bearing)
                print("pos bearing", f)
                valores.append(f)
                gps_world = reconstruction.reference.to_lla(*f)
                print('\nGPS Worldyo bearing', gps_world)
                valores.append(gps_world)
                p1 = shot.pose.get_origin()
                f = r.dot(p1)
                print("pos origin", f)
                gps_world = reconstruction.reference.to_lla(*f)
                print('\nGPS Worldyo origin', gps_world)
                valores.append(f)
                valores.append(gps_world)
