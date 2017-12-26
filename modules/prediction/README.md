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
      inputs are **_perception obstacles_** and **_vehicle localization_**.

  * Evaluator

      Evaluator predicts path and speed separately for any given obstacles. An
      evaluator evaluates a path by outputing a probability for it (lane
      sequence) using the given model stored in _prediction/data/_.

  * Predictor

      Predictor generates predicted trajectories for obstacles. Currently
      supported predictor includes:

      * Lane sequence: obstacle moves following the lanes
      * Free movement: obstacle moves freely
      * Regional movement: obstacle moves in a possible region
