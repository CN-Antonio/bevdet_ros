> fork from [BEVDet-ROS-TensorRT](https://github.com/linClubs/BEVDet-ROS-TensorRT)  
> Will move to [bevdet_vendor](https://github.com/CN-Antonio/bevdet_vendor)

cmake编译安装3.29.2(3.17+)
```
tar zxvf cmake && cd cmake
./bootstrap
make
sudo make install
```

sudo apt install ros-你的ROS版本代号-pcl*

python工具环境
```
pip install ruamel.yaml==0.16.6
```

转换onnx->engine(与本地TensorRT版本匹配)
python tools/export_engine.py cfgs/bevdet_lt_depth.yaml model/img_stage_lt_d.onnx model/bev_stage_lt_d.onnx --postfix="_lt_d_fp16" --fp16=True
