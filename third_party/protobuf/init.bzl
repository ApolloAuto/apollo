"""Loads the protobuf library"""
load("//third_party/protobuf:workspace.bzl", "repo")

def init():
    repo()
    
