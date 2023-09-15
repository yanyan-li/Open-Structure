/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:40:42
 * @LastEditTime: 2023-09-14 16:51:31
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /Open-Structure/baseline/examples/main_entrance.cc
 */
#include <iostream>
#include <string>
#include "src/gui/Interface.hpp"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "usage: ./main_entrance  config.yaml" << std::endl;
        return -1;
    }

    std::cout << std::endl << 
        "\033[0;32mVenom SLAM Simulator Software Copyright (C) 2022 Yanyan Li, Technical University of Munich." << std::endl <<
        "This is a free software that is used for learning, teaching, and testing SLAM strategies." << std::endl <<
        "And you are welcome to contribute it and redistribute it under certain conditions. See LICENSE.txt. \033[0m" << std::endl << std::endl;

    
    // interface
    simulator::Interface venom_entrance("version 2.0.0");
    venom_entrance.StartVenom(argv[1]);

    //
    return 0;
}
