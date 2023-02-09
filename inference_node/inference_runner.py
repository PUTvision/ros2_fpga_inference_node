import vart
import xir
from typing import List
import numpy as np


class InferenceRunner:
    def __init__(self, xmodel_path: str):

        g = xir.Graph.deserialize(xmodel_path)
        subgraphs = self._get_child_subgraph_dpu(g)
        self.subgraphs = self._get_child_subgraph_dpu(g)
        assert len(subgraphs) == 1  # only one DPU kernel

        self.dpu_runner = vart.Runner.create_runner(subgraphs[0], "run")

        input_tensors = self.dpu_runner.get_input_tensors()
        output_tensors = self.dpu_runner.get_output_tensors()
        
        self.input_ndim = tuple(input_tensors[0].dims)
        self.output_ndim = tuple(output_tensors[0].dims)
        
        self.input_buffer = [np.empty(self.input_ndim, dtype=np.float32, order="C")]
        self.output_buffer = [np.empty(self.output_ndim, dtype=np.float32, order="C")]

    # function borrowed from https://github.com/Xilinx/Vitis-AI/blob/master/examples/VART/resnet50_mt_py/resnet50.py#L186
    def _get_child_subgraph_dpu(self, graph: "Graph") -> List["Subgraph"]:
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


    def predict(self, input_img):
        dpu_runner = vart.Runner.create_runner(self.subgraphs[0], "run")
        buffer_run = self.input_buffer[0]
        buffer_run[0, ...] = input_img.reshape(self.input_ndim[1:])

        job_id = dpu_runner.execute_async(self.input_buffer, self.output_buffer)
        dpu_runner.wait(job_id)

        output_img = self.output_buffer[0].reshape(self.output_ndim[1:])
        return output_img
