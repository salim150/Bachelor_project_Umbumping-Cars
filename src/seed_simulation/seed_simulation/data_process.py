# class to process the data from the simulation

import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt


class DataProcessor:
    def __init__(self, robot_num, file_name):
        self.robot_num = robot_num
        self.file_name = file_name

    def calculate_path_length(self, trajectory, i):
        """
        Calculate the path length of the trajectory or robot i
        :param trajectory: the trajectory
        :return: the path length
        """
        x_list = trajectory[0, i, :]
        y_list = trajectory[1, i, :]

        path_length = 0.0

        initial_x = x_list[0]
        initial_y = y_list[0]

        assert len(x_list) == len(y_list)

        for idx in range(len(x_list)):

            x = x_list[idx]
            y = y_list[idx]

            dx = x - initial_x
            dy = y - initial_y

            initial_x = x
            initial_y = y

            path_length += np.sqrt(dx ** 2 + dy ** 2)
        
        return path_length
    
    def calculate_avg_path_length(self, trajectory):
        """
        Calculate the average path length of the trajectory over all the robots
        :param trajectory: the trajectory
        :return: the average path length
        """
        print("Calculating Path Length...")
        path_length = 0
        for i in range(self.robot_num):
            path_length += self.calculate_path_length(trajectory, i)
        return path_length / self.robot_num
    
    def calculate_acceleration_usage(self, trajectory, i):
        """
        Calculate the acceleration usage of the trajectory or robot i
        :param trajectory: the trajectory
        :return: the acceleration usage
        """
        a = trajectory[4, i, :]
        return np.sum(np.abs(a))
    
    def calculate_avg_acceleration_usage(self, trajectory):
        """
        Calculate the average acceleration usage of the trajectory over all the robots
        :param trajectory: the trajectory
        :return: the average acceleration usage
        """
        print("Calculating Acceleration Usage...")
        acceleration_usage = 0
        for i in range(self.robot_num):
            acceleration_usage += self.calculate_acceleration_usage(trajectory, i)
        return acceleration_usage / self.robot_num
    
    def calculate_steering_usage(self, trajectory, i):
        """
        Calculate the steering usage of the trajectory or robot i
        :param trajectory: the trajectory
        :return: the steering usage
        """
        steering = trajectory[5, i, :]
        return np.sum(np.abs(steering))
    
    def calculate_avg_steering_usage(self, trajectory):
        """
        Calculate the average steering usage of the trajectory over all the robots
        :param trajectory: the trajectory
        :return: the average steering usage
        """
        print("Calculating Steering Usage...")
        steering_usage = 0
        for i in range(self.robot_num):
            steering_usage += self.calculate_steering_usage(trajectory, i)
        return steering_usage / self.robot_num
    
    def calculate_speed_avg(self, trajectory, i):
        """
        Calculate the average speed of the trajectory or robot i
        :param trajectory: the trajectory
        :return: the average speed
        """
        v = trajectory[3, i, :]
        return np.mean(v)
    
    def calculate_avg_speed(self, trajectory):
        """
        Calculate the average speed of the trajectory over all the robots
        :param trajectory: the trajectory
        :return: the average speed
        """
        print("Calculating Average Speed...")
        speed = 0
        for i in range(self.robot_num):
            speed += self.calculate_speed_avg(trajectory, i)
        return speed / self.robot_num
    
    def calculate_avg_computational_time(self, computational_time):
        """
        Calculate the computational time of the trajectory
        :param computational_time: the computational time
        :return: the computational time
        """
        print("Calculating Average Computational Time...")
        return sum(computational_time) / len(computational_time)
    
    def calculate_initial_dist(self, trajectory, i):
        """
        Calculate the initial distance of the trajectory or robot i
        :param trajectory: the trajectory
        :return: the initial distance
        """
        x = trajectory[0, i, :]
        y = trajectory[1, i, :]
        return np.sqrt(x[0] ** 2 + y[0] ** 2)
    
    def calculate_avg_initial_dist(self, trajectory):
        """
        Calculate the average initial distance of the trajectory over all the robots
        :param trajectory: the trajectory
        :return: the average initial distance
        """
        print("Calculating Initial Distance...")
        initial_dist = 0
        for i in range(self.robot_num):
            initial_dist += self.calculate_initial_dist(trajectory, i)
        return initial_dist / self.robot_num

    def post_process_simultation(self, trajectory, computational_time, method):
        """
        Post process the simulation
        :param trajectory: the trajectory
        :param computational_time: the computational time
        :return: the post processed data
        """
        avg_path_length = self.calculate_avg_path_length(trajectory)
        acceleration_usage = self.calculate_avg_acceleration_usage(trajectory)
        steering_usage = self.calculate_avg_steering_usage(trajectory)
        avg_speed = self.calculate_avg_speed(trajectory)
        avg_computational_time = self.calculate_avg_computational_time(computational_time)
        avg_initial_dist = self.calculate_avg_initial_dist(trajectory)

        data = {
            "Path Length": avg_path_length,
            "Acceleration Usage": acceleration_usage,
            "Steering Usage": steering_usage,
            "Average Speed": avg_speed,
            "Avg Computational Time": avg_computational_time,
            "Initial Distance": avg_initial_dist,
            "Robot Number": self.robot_num,
            "File Name": self.file_name,
            "Method": method
        }

        print("Data Processed Successfully!\n")
        return data
    
    def remove_df_duplicates(self, df):
        """
        Remove the duplicates from the dataframe
        :param df: the dataframe
        :return: the dataframe without duplicates
        """
        return df.drop_duplicates(subset=["Robot Number", "File Name", "Method"], keep="last")