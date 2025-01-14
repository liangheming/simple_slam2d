#include "commons.h"

std::vector<std::pair<int, int>> draw_line(const float &x0f, const float &y0f, const float &x1f, const float &y1f)
{
    std::vector<std::pair<int, int>> line;
    int x0 = static_cast<int>(floorf(x0f)), y0 = static_cast<int>(floorf(y0f)), x1 = static_cast<int>(floorf(x1f)), y1 = static_cast<int>(floorf(y1f));

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    line.reserve(dx - dy);
    int err = dx + dy, e2; /* error value e_xy */

    for (;;)
    { /* loop */
        line.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        } /* e_xy+e_x > 0 */
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        } /* e_xy+e_y < 0 */
    }
    return line;
}