# 实验测试记录
仿真启动已经保存成excel文件在`test.py`

实验数据excel保存与`results_seaborn/`

时间线
---
24-01-21
实验设置
```python
sim_time = 70
x0 = np.array([0, 0, 0.0])   
goal = [6.5, 0, 0.0] 
moving_obs = [(-1.5, 5.5, 0.0, -0.35, 0.3)] #4s
moving_obs = [(-1.0, 5.5, 0.0, -0.35, 0.3)] #5s
moving_obs = [(-0.5, 5.5, 0.0, -0.35, 0.3)] #5s
```

24-01-22
实验设置
```python
sim_time = 80
x0 = np.array([0, 0, 0.0])   
goal = [6.0, 0, 0.0] 
moving_obs = [(-1.5, 5.5, 0.0, -0.35, 0.3)] #4.5s
moving_obs = [(-1.0, 5.5, 0.0, -0.35, 0.3)] #4.5s
moving_obs = [(-0.5, 5.5, 0.0, -0.35, 0.3)] #4.5s
```