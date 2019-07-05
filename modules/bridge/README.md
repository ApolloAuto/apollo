# Bridge

## Introduction
  This module provides a way for other Apollo modules interactiving with process outside of Apollo by socket.
  It includes sender and receiver components.

## Input
  In sender component, there is only one input, which is the proto struct sender handled.
  In receiver comonent, its input is different with others. Its input comes from UDP socket.

## Output
  Sender's output is by way of UDP socket.
  Receiver has one output which is receiver handled.

