#include "raycast.h"
bool raycast(Sailbot::Map& map, int y1, int x1, int y2, int x2)
{
http://eugen.dedu.free.fr/projects/bresenham/
    int i;               // loop counter
    int ystep, xstep;    // the step on y and x axis
    int error;           // the error accumulated during the increment
    int errorprev;       // *vision the previous value of the error variable
    int y = y1, x = x1;  // the line points
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    if (!map.isWalkable(y1, x1))
        return false;// first point
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0) {
        ystep = -1;
        dy = -dy;
    }
    else
        ystep = 1;
    if (dx < 0) {
        xstep = -1;
        dx = -dx;
    }
    else
        xstep = 1;
    ddy = 2 * dy;  // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy) {  // first octant (0 <= slope <= 1)
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx;  // start in the middle of the square
        for (i = 0; i < dx; i++) {  // do not use the first point (already done)
            x += xstep;
            error += ddy;
            if (error > ddx) {  // increment y if AFTER the middle ( > )
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx)  // bottom square also
                    if (!map.isWalkable(y - ystep, x))
                        return false;
                    else if (error + errorprev > ddx)  // left square also
                        if (!map.isWalkable(y, x - xstep))
                            return false;
                        else {  // corner: bottom and left squares also
                            if (!map.isWalkable(y - ystep, x))
                                return false;
                            if (!map.isWalkable(y, x - xstep))
                                return false;
                        }
            }
            if (!map.isWalkable(y, x))
                return false;
            errorprev = error;
        }
    }
    else {  // the same as above
        errorprev = error = dy;
        for (i = 0; i < dy; i++) {
            y += ystep;
            error += ddx;
            if (error > ddy) {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                    if (!map.isWalkable(y, x - xstep))
                        return false;
                    else if (error + errorprev > ddy)
                        if (!map.isWalkable(y - ystep, x))
                            return false;
                        else {
                            return !map.isWalkable(y, x - xstep);
                            if (!map.isWalkable(y - ystep, x))
                                return false;
                        }
            }
            if (!map.isWalkable(y, x))
                return false;
            errorprev = error;
        }
    }
    return true;
    // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
}