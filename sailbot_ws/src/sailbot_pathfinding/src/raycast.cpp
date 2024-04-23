#include "raycast.h"
bool raycast(Map &map, int x1, int y1, int x2, int y2)
{
http:                   // eugen.dedu.free.fr/projects/bresenham/
    int i;              // loop counter
    int ystep, xstep;   // the step on y and x axis
    int error;          // the error accumulated during the increment
    int errorprev;      // *vision the previous value of the error variable
    int y = y1, x = x1; // the line points
    int ddy, ddx;       // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    if (!map.isWalkable(x1, y1, RAYCAST_BLOCKED_CUTOFF))
        return false; // first point
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0)
    {
        ystep = -1;
        dy = -dy;
    }
    else
        ystep = 1;
    if (dx < 0)
    {
        xstep = -1;
        dx = -dx;
    }
    else
        xstep = 1;
    ddy = 2 * dy; // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy)
    { // first octant (0 <= slope <= 1)
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx; // start in the middle of the square
        for (i = 0; i < dx; i++)
        { // do not use the first point (already done)
            x += xstep;
            error += ddy;
            if (error > ddx)
            { // increment y if AFTER the middle ( > )
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx)
                { // bottom square also
                    if (!map.isWalkable(x, y - ystep, RAYCAST_BLOCKED_CUTOFF))
                    {
                        return false;
                    }
                }
                else if (error + errorprev > ddx)
                { // left square also
                    if (!map.isWalkable(x - xstep, y, RAYCAST_BLOCKED_CUTOFF))
                    {
                        return false;
                    }
                }
                else
                { // corner: bottom and left squares also
                    if (!map.isWalkable(x, y - ystep, RAYCAST_BLOCKED_CUTOFF))
                        return false;
                    if (!map.isWalkable(x - xstep, y, RAYCAST_BLOCKED_CUTOFF))
                        return false;
                }
            }
            if (!map.isWalkable(x, y, RAYCAST_BLOCKED_CUTOFF))
                return false;
            errorprev = error;
        }
    }
    else
    { // the same as above
        errorprev = error = dy;
        for (i = 0; i < dy; i++)
        {
            y += ystep;
            error += ddx;
            if (error > ddy)
            {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                {
                    if (!map.isWalkable(x - xstep, y, RAYCAST_BLOCKED_CUTOFF))
                    {
                        return false;
                    }
                }
                else if (error + errorprev > ddy)
                {
                    if (!map.isWalkable(x, y - ystep, RAYCAST_BLOCKED_CUTOFF))
                    {
                        return false;
                    }
                }
                else
                {
                    if (!map.isWalkable(x - xstep, y, RAYCAST_BLOCKED_CUTOFF))
                        return false;
                    if (!map.isWalkable(x, y - ystep, RAYCAST_BLOCKED_CUTOFF))
                        return false;
                }
            }
            if (!map.isWalkable(x, y, RAYCAST_BLOCKED_CUTOFF))
                return false;
            errorprev = error;
        }
    }
    return true;
    // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
}

float accumulate_danger_raycast(Map &map, int x1, int y1, int x2, int y2)
{
http:                   // eugen.dedu.free.fr/projects/bresenham/
    int i;              // loop counter
    int ystep, xstep;   // the step on y and x axis
    int error;          // the error accumulated during the increment
    int errorprev;      // *vision the previous value of the error variable
    int y = y1, x = x1; // the line points
    int ddy, ddx;       // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    float total = 0;
    total += map.valueAt(x1, y1);
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0)
    {
        ystep = -1;
        dy = -dy;
    }
    else
        ystep = 1;
    if (dx < 0)
    {
        xstep = -1;
        dx = -dx;
    }
    else
        xstep = 1;
    ddy = 2 * dy; // work with double values for full precision
    ddx = 2 * dx;
    if (ddx >= ddy)
    { // first octant (0 <= slope <= 1)
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx; // start in the middle of the square
        for (i = 0; i < dx; i++)
        { // do not use the first point (already done)
            x += xstep;
            error += ddy;
            if (error > ddx)
            { // increment y if AFTER the middle ( > )
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx) // bottom square also
                    total += map.valueAt(x, y - ystep);
                else if (error + errorprev > ddx) // left square also
                    total += map.valueAt(x - xstep, y);
                else
                { // corner: bottom and left squares also
                    total += map.valueAt(x, y - ystep);
                    total += map.valueAt(x - xstep, y);
                }
            }
            total += map.valueAt(x, y);
            errorprev = error;
        }
    }
    else
    { // the same as above
        errorprev = error = dy;
        for (i = 0; i < dy; i++)
        {
            y += ystep;
            error += ddx;
            if (error > ddy)
            {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                    total += map.valueAt(x - xstep, y);
                else if (error + errorprev > ddy)
                    total += map.valueAt(x, y - ystep);
                else {
                    total += map.valueAt(x - xstep, y);
                    total += map.valueAt(x, y - ystep);
                }
            }
            total += map.valueAt(x, y);
            errorprev = error;
        }
    }
    return total;
    // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm
}