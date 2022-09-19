from ctypes import *
from typing import List
import cv2
import numpy as np
import xir
import vart
import os
import math
import threading
import time
import sys


class Resnet50Engine:
    def __init__(self, model_path: str, name_path: str):
        self.MEANS = [104.0, 107.0, 123.0]
        self.SCALES = [1.0, 1.0, 1.0]
        self.shape = [224, 224] # w,h

        self.g = xir.Graph.deserialize(model_path)
        self.subgraphs = self.get_child_subgraph_dpu(self.g)

        self.dpu_runner = vart.Runner.create_runner(self.subgraphs[0], "run")
        self.input_fixpos = self.dpu_runner.get_input_tensors()[0].get_attr("fix_point")
        self.input_scale = 2**self.input_fixpos

        self.inputTensors = self.dpu_runner.get_input_tensors()
        self.outputTensors = self.dpu_runner.get_output_tensors()
        self.input_ndim = tuple(self.inputTensors[0].dims)
        self.pre_output_size = int(self.outputTensors[0].get_data_size() / self.input_ndim[0])

        self.output_ndim = tuple(self.outputTensors[0].dims)
        self.output_fixpos = self.outputTensors[0].get_attr("fix_point")
        self.output_scale = 1 / (2**self.output_fixpos)

        self.classes = eval(open(name_path).read())

    def cls_to_name(self, id: int):
        return self.classes[id]

    def preprocess(self, image):
        image = cv2.resize(image, self.shape)
        image = (image - self.MEANS) * self.SCALES * self.input_scale
        image = image.astype(np.int8)
        return image

    def infer(self, image):
        inp = self.preprocess(image)
        outs = self.run(inp)

        return self.postprocess(outs)

    def postprocess(self, outs):
        outs = np.squeeze(outs)
        outs = self.CPUCalcSoftmax(outs, self.output_scale)
        outs = np.argmax(outs)
        return outs

    def run(self, img):
        inputData = [np.empty(self.input_ndim, dtype=np.int8, order="C")]
        outputData = [np.empty(self.output_ndim, dtype=np.int8, order="C")]

        inputData[0] = img.reshape(self.input_ndim[1:])

        job_id = self.dpu_runner.execute_async(inputData, outputData)
        self.dpu_runner.wait(job_id)

        return outputData


    @staticmethod
    def CPUCalcSoftmax(x, scale):
        x = x * scale
        e_x = np.exp(x - np.max(x))
        return e_x / e_x.sum(axis=0)

    @staticmethod
    def get_child_subgraph_dpu(graph: "Graph") -> List["Subgraph"]:
        assert graph is not None, "'graph' should not be None."
        root_subgraph = graph.get_root_subgraph()
        assert (root_subgraph
                is not None), "Failed to get root subgraph of input Graph object."
        if root_subgraph.is_leaf:
            return []
        child_subgraphs = root_subgraph.toposort_child_subgraph()
        assert child_subgraphs is not None and len(child_subgraphs) > 0
        return [
            cs for cs in child_subgraphs
            if cs.has_attr("device") and cs.get_attr("device").upper() == "DPU"
        ]
