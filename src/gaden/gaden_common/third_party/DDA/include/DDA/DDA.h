#include <vector>
#include "../../third_party/glm/glm/glm.hpp"
#include "../../third_party/glm/glm/common.hpp"
#include <functional>
#include <iostream>

static void resetColor()
{
    printf("\033[0m");
}

static void Error()
{
    printf("\033[1;31m");
    printf("[ERROR] ");
    resetColor();
}

static void Warn()
{
    printf("\033[1;33m");
    printf("[WARN] ");
    resetColor();
}

namespace DDA::_2D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<glm::ivec2, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<glm::ivec2, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    template <typename T> struct Map
    {
        Map(){}
        Map(std::vector<std::vector<T>> _cells,
            glm::vec2 _origin,
            float _resolution): cells(_cells), origin(_origin), resolution(_resolution)
            {}
        std::vector<std::vector<T>> cells;
        glm::vec2 origin;
        float resolution;
    };

    // returns true if a blocked cell was hit. The outline of the map is considered blocked.
    template <typename T>
    RayCastInfo castRay(
        const glm::vec2& start, glm::vec2 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec2&)>& positionPredicate = [](const glm::vec2& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false, 0};
        }

        glm::ivec2 currentCell = (start - map.origin) / map.resolution;
        glm::vec2 currentPosition = start;
        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray outside the environment!\n");
            return {false, 0};
        }

        direction = direction / glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);

        float currentDistance = 0;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }

            if ((stepX != 0 && tX > 0 && tX < tY) || (stepY == 0 || tY <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            currentCell = glm::floor((currentPosition - map.origin) / map.resolution);

            if (glm::length(currentPosition - start) > maxDistance)
                return {false, maxDistance};
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }

    // returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(
        const glm::vec2& start, glm::vec2 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec2&)>& positionPredicate = [](const glm::vec2& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }

        glm::ivec2 currentCell = (start - map.origin) / map.resolution;
        glm::vec2 currentPosition = start;

        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f)\n", start.x, start.y);

            return RayMarchInfo();
        }

        direction = direction / glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);

        float currentDistance = 0;
        std::vector<std::pair<glm::ivec2, float>> lengthsMap;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }

            if ((stepX != 0 && tX > 0 && tX < tY) || (stepY == 0 || tY <= 0))
            {
                lengthsMap.emplace_back(currentCell, tX);
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else
            {
                lengthsMap.emplace_back(currentCell, tY);
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            currentCell = glm::floor((currentPosition - map.origin) / map.resolution);

            if (glm::length(currentPosition - start) > maxDistance)
                return RayMarchInfo();
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y]) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_2D

namespace DDA::_3D
{
    struct RayCastInfo
    {
        bool hitSomething;
        float distance;
    };

    struct RayMarchInfo
    {
        std::vector<std::pair<glm::ivec3, float>> lengthInCell;
        float totalLength;
        RayMarchInfo() : totalLength(0)
        {}
        RayMarchInfo(std::vector<std::pair<glm::ivec3, float>> inputMap, float length) : lengthInCell(std::move(inputMap)), totalLength(length)
        {}
    };

    template <typename T> struct Map
    {
        Map(){}
        Map(std::vector<std::vector<std::vector<T>>> _cells,
            glm::vec3 _origin,
            float _resolution): cells(_cells), origin(_origin), resolution(_resolution)
            {}
        std::vector<std::vector<std::vector<T>>> cells;
        glm::vec3 origin;
        float resolution;
    };

    // returns true if a blocked cell was hit. The outline of the map is considered blocked.
    template <typename T>
    RayCastInfo castRay(
        const glm::vec3& start, glm::vec3 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec3&)>& positionPredicate = [](const glm::vec3& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");
            return {false, 0};
        }

        glm::ivec3 currentCell = (start - map.origin) / map.resolution;
        glm::vec3 currentPosition = start;
        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() || !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) ||
            !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray origin in invalid position: (%f, %f, %f)\n", start.x, start.y, start.z);

            return {false, 0};
        }

        direction = direction / glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);
        int stepZ = glm::sign(direction.z);

        float currentDistance = 0;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;
            float zCoordNext = (stepZ > 0 ? currentCell.z + 1 : currentCell.z) * map.resolution + map.origin.z;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }
            float tZ = (zCoordNext - currentPosition.z) / direction.z;
            if (tZ <= 0)
            {
                zCoordNext += stepZ * map.resolution;
                tZ = (zCoordNext - currentPosition.z) / direction.z;
            }

            if ((stepX != 0 && tX > 0) && (tX < tY || stepY == 0 || tY <= 0) && (tX < tZ || stepZ == 0 || tZ <= 0))
            {
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if ((stepY != 0 && tY > 0) && (tY < tZ || stepZ == 0 || tZ <= 0))
            {
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = glm::floor((currentPosition - map.origin) / map.resolution);

            if (currentDistance > maxDistance)
                return {false, maxDistance};
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) || !positionPredicate(currentPosition))
                return {true, currentDistance};
        }
    }

    // returns how far through each cell the ray has traveled. Useful for volumetric calculations
    template <typename T>
    RayMarchInfo marchRay(
        const glm::vec3& start, glm::vec3 direction, const float maxDistance, const Map<T>& map, const std::function<bool(const T&)>& mapPredicate,
        const std::function<bool(const glm::vec3&)>& positionPredicate = [](const glm::vec3& v) { return true; })
    {
        if (glm::length(direction) == 0)
        {
            Warn();
            printf("Ray of length 0\n");

            return RayMarchInfo();
        }

        glm::ivec3 currentCell = (start - map.origin) / map.resolution;
        glm::vec3 currentPosition = start;

        if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
            currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() || !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) ||
            !positionPredicate(currentPosition))
        {
            Error();
            printf("Ray outside the environment!\n");

            return RayMarchInfo();
        }

        direction = direction / glm::length(direction);
        int stepX = glm::sign(direction.x);
        int stepY = glm::sign(direction.y);
        int stepZ = glm::sign(direction.z);

        float currentDistance = 0;
        std::vector<std::pair<glm::ivec3, float>> lengthsMap;
        while (true)
        {
            float xCoordNext = (stepX > 0 ? currentCell.x + 1 : currentCell.x) * map.resolution + map.origin.x;
            float yCoordNext = (stepY > 0 ? currentCell.y + 1 : currentCell.y) * map.resolution + map.origin.y;
            float zCoordNext = (stepZ > 0 ? currentCell.z + 1 : currentCell.z) * map.resolution + map.origin.z;

            // how far to move along direction, correcting for floating-point shenanigans
            float tX = (xCoordNext - currentPosition.x) / direction.x;
            if (tX <= 0)
            {
                xCoordNext += stepX * map.resolution;
                tX = (xCoordNext - currentPosition.x) / direction.x;
            }
            float tY = (yCoordNext - currentPosition.y) / direction.y;
            if (tY <= 0)
            {
                yCoordNext += stepY * map.resolution;
                tY = (yCoordNext - currentPosition.y) / direction.y;
            }
            float tZ = (zCoordNext - currentPosition.z) / direction.z;
            if (tZ <= 0)
            {
                zCoordNext += stepZ * map.resolution;
                tZ = (zCoordNext - currentPosition.z) / direction.z;
            }

            if (stepX != 0 && (tX < tY || stepY == 0) && (tX < tZ || stepZ == 0))
            {
                if (tX > 0)
                    lengthsMap.emplace_back(currentCell, tX);
                currentPosition += direction * tX;
                currentDistance += tX;
            }
            else if (stepY != 0 && (tY < tZ || stepZ == 0))
            {
                if (tY > 0)
                    lengthsMap.emplace_back(currentCell, tY);
                currentPosition += direction * tY;
                currentDistance += tY;
            }
            else
            {
                if (tZ > 0)
                    lengthsMap.emplace_back(currentCell, tZ);
                currentPosition += direction * tZ;
                currentDistance += tZ;
            }

            currentCell = glm::floor((currentPosition - map.origin) / map.resolution);

            if (currentDistance > maxDistance)
                return RayMarchInfo();
            else if (currentCell.x < 0 || currentCell.x >= map.cells.size() || currentCell.y < 0 || currentCell.y >= map.cells[0].size() ||
                     currentCell.z < 0 || currentCell.z >= map.cells[0][0].size() ||
                     !mapPredicate(map.cells[currentCell.x][currentCell.y][currentCell.z]) || !positionPredicate(currentPosition))
                return {lengthsMap, currentDistance};
        }
    }
} // namespace DDA::_3D
