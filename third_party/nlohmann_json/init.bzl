"""Loads the nlohmann_json library"""
load("//third_party/nlohmann_json:workspace.bzl", "repo")

def init():
    repo()
    
