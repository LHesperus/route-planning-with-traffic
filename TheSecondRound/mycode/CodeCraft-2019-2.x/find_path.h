#pragma once
#include "iostream"
#include <sstream>
#include <fstream>
#include <string>

#include <math.h>
#include <cmath>
#include "def_base_Class.h"
#include "def_base_value.h"
extern int sum_path;//总路数,注意车量级，防止溢出
void min_time_Dijkstra(int p_start,int p_end,int path_a_b[n_path],int car_speed,int car_N,Cross Cross_group[n_cross],Road Road_group[n_road],Car Car_group[n_car]);
int Is_have_loop(int p_start,int p_end,Cross Cross_group[n_cross],Road Road_group[n_road]);