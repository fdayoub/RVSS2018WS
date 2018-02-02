from __future__ import print_function
import matplotlib.pyplot as plt
import cv2
from threading import Thread
import time
import numpy as np
import sys
sys.path.append('../thirdparties/gtsam/build/cython')
import gtsam



class Slam(object):
	# assuming x, y, theta and delx, dely, deltheta
	def __init__(self, odom_global,odom_relative,visual_global,visual_relative):
		self.odom_global = self.point_to_pose(odom_global)
		self.odom_relative = self.point_to_pose(odom_relative)
		self.visual_global = self.point_to_pose(visual_global)
		self.visual_relative = self.point_to_pose(visual_relative)

		# print(odom_global)
		# self.draw_trajectories([self.odom_global, self.visual_global], ['b', 'r'], 2)
		# exit()

		# print(np.asarray(visual_global))
		# exit()
		# print(odom_relative)
		# print(visual_global)
		# print(visual_relative)

		# shorthand symbols:
		self.X = lambda i: int(gtsam.symbol(ord('x'), i))

	def point_to_pose(self, points):
		# poses = [gtsam.Pose2(point[0], point[1], point[2]) for point in points]
		poses = []
		for point in points:
			pose = gtsam.Pose2(point[0], point[1], np.deg2rad(point[2]))
			poses.append(pose)

		return poses

	def estimate_global(self):
		np.random.seed(2)
		n0 = 0.000001
		n03 = np.array([n0, n0, n0])
		nNoiseFactor3 = np.array([0.2, 0.2, 0.2]) # TODO: something about floats and major row? check cython

		# Create an empty nonlinear factor graph
		graph = gtsam.NonlinearFactorGraph()

		# Add a prior on the first pose, setting it to the origin
		priorMean = self.odom_global[0]
		priorNoise = gtsam.noiseModel_Diagonal.Sigmas(n03)
		graph.add(gtsam.PriorFactorPose2(self.X(1), priorMean, priorNoise))

		# Add odometry factors
		odometryNoise = gtsam.noiseModel_Diagonal.Sigmas(nNoiseFactor3)
		for i, pose in enumerate(self.odom_relative):
			graph.add(gtsam.BetweenFactorPose2(self.X(i+1), self.X(i+2), pose, odometryNoise))

		# Add visual factors
		visualNoise = gtsam.noiseModel_Diagonal.Sigmas(nNoiseFactor3)
		for i, pose in enumerate(self.visual_relative):
			graph.add(gtsam.BetweenFactorPose2(self.X(i+1), self.X(i+2), pose, visualNoise))

		# set initial guess to odometry estimates
		initialEstimate = gtsam.Values()
		for i, pose in enumerate(self.odom_global):
			initialEstimate.insert(self.X(i+1), pose)

		# optimize using Levenberg-Marquardt optimization
		params = gtsam.LevenbergMarquardtParams()
		params.setVerbosityLM("SUMMARY")
		# gtsam_quadrics.setMaxIterations(params, iter)
		# gtsam_quadrics.setRelativeErrorTol(params, 1e-8)
		# gtsam_quadrics.setAbsoluteErrorTol(params, 1e-8)
		
		# graph.print_('graph')
		# initialEstimate.print_('initialEstimate ')
		optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate, params)

		# parameters = gtsam.GaussNewtonParams()
		# parameters.relativeErrorTol = 1e-8
		# parameters.maxIterations = 300
		# optimizer = gtsam.GaussNewtonOptimizer(graph, initialEstimate, parameters)

		result = optimizer.optimize()

		# result.print_('result ')
		# self.draw_trajectories([self.odom_global, self.visual_global], ['b', 'r'], 2)

		return self.unwrap_results(result)



	def unwrap_results(self, result):
		poses = []
		for i in range(result.keys().size()):
			pose = result.atPose2(self.X(i+1))
			poses.append([pose.x(), pose.y(), np.rad2deg(pose.theta())])

		return poses

	def draw_trajectories(self, results, colours, fig_n):
		# Plot resulting poses (dicts, gtsams, result)
		XYZs = []
		for result in results:
			XYZ = []
			if not hasattr(result,'keys') and hasattr(result[4],'translation'):
				for measurement in result:
					trans = measurement.translation().vector()
					XYZ.append(trans)
				XYZs.append(np.asarray(XYZ))
			elif hasattr(result,'keys'):
				for i in range(result.keys().size()):
					pose_i = result.atPose2(self.X(i+1))
					trans = pose_i.translation().vector()
					XYZ.append(trans)
					# print(trans)
				XYZs.append(np.asarray(XYZ))
		XYZs = np.asarray(XYZs)

		# plot GT, noisy, est
		fig = plt.figure(fig_n)
		ax = fig.gca()
		ax.clear()
		ax.grid(True)
		for XYZ, colour in zip(XYZs, colours):
			ax.plot(XYZ[:,0], XYZ[:,1], colour)

		# print(XYZs)

		# scale axis
		# max_range = (XYZs.max((0,1))-XYZs.min((0,1))).max() / 2.0
		# XYZs_mid = (XYZs.max((0,1))+XYZs.min((0,1))) * 0.5
		# ax.set_xlim(XYZs_mid[0] - max_range, XYZs_mid[0] + max_range)
		# ax.set_ylim(XYZs_mid[1] - max_range, XYZs_mid[1] + max_range)
		# ax.set_zlim(XYZs_mid[2] - max_range, XYZs_mid[2] + max_range)

		plt.show()
