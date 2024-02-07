import numpy as np

def sawtooth(x):
    x = x+np.pi
    return ((x + np.pi) % (2 * np.pi) - np.pi)/np.pi  # or equivalently   2*arctan(tan(x/2))


def coord2cart(coords,coords_ref=(48.19924,-3.01461)):
    '''
    in :
        coords = (lat,lon)
        coords_ref = centre du plan
    return :
        x_tilde et y_tilde coord sur le plan centre en coord_ref
    '''

    ly,lx = coords
    lym,lxm = coords_ref

    x_tilde = 6378000 * cos(ly*np.pi/180)*(lx-lxm)*np.pi/180
    y_tilde = 6378000 * (ly-lym)*np.pi/180

    return np.array([[x_tilde,y_tilde]]).T

def is_waypoint_passed(next_wp_coord,previous_wp_coord,cm):

    xy_nwp = coord2cart(next_wp_coord)
    xy_pwp = coord2cart(previous_wp_coord)
    xy_m = coord2cart(cm)

    return (((xy_nwp-xy_pwp)@(xy_m-xy_nwp).T)[0,0] > 0)