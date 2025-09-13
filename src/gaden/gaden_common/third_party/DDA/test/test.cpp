#include "../include/DDA/DDA.h"

#include <iostream>

void test()
{
    std::cout<<"Expected result:\n"
    "2D\n"
    "Raycast:\n"
    "1 4.5m\n"
    "Raymarch:\n"
    "(2,0): 0.424264\n"
    "(3,0): 0.282843\n"
    "(3,1): 1.13137\n"
    "(4,1): 0.282843\n"
    "(4,2): 1.13137\n"
    "3D\n"
    "Raycast:\n"
    "1 8.66025m\n"
    "Raymarch:\n"
    "(0,0): 1.73205\n"
    "(1,1): 1.73205\n"
    "(2,2): 1.73205\n"
    "(3,3): 1.73205\n"
    "(4,4): 1.73205\n";

    std::cout<<"\n\nActual result:\n";
    
    //2D
    {
        std::cout<<"2D\n";

        DDA::_2D::Map<bool> map;
        map.cells = {
            {1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 1, 1, 1}
        };
        map.origin = {0,0};
        map.resolution = 1;

        std::cout<<"Raycast:\n";
        {
            auto result = DDA::_2D::castRay<bool>({0.5, 0.5}, {0,1}, 10.0f, map, [](bool b){return b;});
            std::cout<<result.hitSomething<<" "<<result.distance<<"m\n";
        }

        std::cout<<"Raymarch:\n";
        {
            auto result = DDA::_2D::marchRay<bool>({2.7, 0.5}, {1,1}, 10, map,[](bool b){return b;});
            for(const auto& p : result.lengthInCell)
            {
                glm::ivec2 cell =p.first;
                std::cout<<"("<<cell.x<<","<<cell.y<<")"<<": "<<p.second<<"\n";
            }
        }
    }

    //3D
    {
        std::cout<<"3D\n";
        DDA::_3D::Map<bool> map;
        map.origin = {0,0,0};
        map.resolution = 1;
        map.cells = std::vector<std::vector<std::vector<bool>>> (5, std::vector<std::vector<bool>>(5, std::vector<bool>(5, true)) );
        std::cout<<"Raycast:\n";
        {
            auto result = DDA::_3D::castRay<bool>({0, 0, 0}, {1, 1, 1}, 10, map, [](bool b){return b;});
            std::cout<<result.hitSomething<<" "<<result.distance<<"m\n";
        }

        std::cout<<"Raymarch:\n";
        {
            auto result = DDA::_3D::marchRay<bool>({0, 0, 0}, {1,1,1}, 10, map,[](bool b){return b;});
            for(const auto& p : result.lengthInCell)
            {
                glm::ivec2 cell =p.first;
                std::cout<<"("<<cell.x<<","<<cell.y<<")"<<": "<<p.second<<"\n";
            }
            
        }
    }
}

int main()
{
    test();
    return 0; 
}