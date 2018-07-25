# Prediction

## Introduction
  The prediction module receives the obstacles from the perception module with
  their basic perception information including positions, headings, velocities,
  accelerations, and generates the predicted trajectories with probabilities for
  the obstacles.

## Input
  * Obstacles from perception module
  * Localizaton from localization module

## Output
  * Obstacles additionally with predicted trajectories

## Functionalities
  * Container

      Container stores input data from subscribed channels. Current supported
      inputs are **_perception obstacles_**, **_vehicle localization_** and **_vehicle planning_**.

  * Evaluator

      Evaluator predicts path and speed separately for any given obstacles. An
      evaluator evaluates a path by outputing a probability for it (lane
      sequence) using the given model stored in _prediction/data/_.

      Three types of evaluators will be provided including:

      * Cost evaluator: probability is calculated by a set of cost functions

      * MLP evaluator: probability is calculated with an MLP model

      * RNN evaluator: probability is calculated with an RNN model

  * Predictor

      Predictor generates predicted trajectories for obstacles. Currently
      supported predictor includes:

      * Empty: obstacles have no predicted trajectories
      * Single lane: Obstacles move along a single lane in highway navigation mode. Obstacles not on lane will be ignored.
      * Lane sequence: obstacle moves along the lanes
      * Move sequence: obstacle moves along the lanes by following its kinetic pattern
      * Free movement: obstacle moves freely
      * Regional movement: obstacle moves in a possible region
