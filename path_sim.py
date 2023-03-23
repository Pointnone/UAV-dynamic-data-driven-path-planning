from pyproj import Geod

def simpleSimulate(waypoints, speed = 15.0, temporal_res = 1.0):
    g = Geod(ellps="WGS84")

    sim_points = [waypoints[0]]
    zipped_points = [[waypoints[i], waypoints[i+1]] for i in range(len(waypoints)-1)]

    init_lons = [z[0][0] for z in zipped_points]
    init_lats = [z[0][1] for z in zipped_points]
    term_lons = [z[1][0] for z in zipped_points]
    term_lats = [z[1][1] for z in zipped_points]

    _, _, dists = g.inv(init_lons, init_lats, term_lons, term_lats)
    num_points = [d/(speed*temporal_res) for d in dists]
    
    for i in range(len(zipped_points)):
        z = zipped_points[i]
        res = g.inv_intermediate(z[0][0], z[0][1], z[1][0], z[1][1], num_points[i], terminus_idx=0)
        res_points = [(lon, lat) for lon,lat in zip(res.lons, res.lats)]

        sim_points.extend(res_points)
    return sim_points

if(__name__ == "__main__"):
    w = [(10.0, 30.0), (10.1, 30.0)]
    dists = simpleSimulate(w, 15.0, 1.0)
    print(dists)