import tensorflow as tf

# 1. 基础检测
print(f"TensorFlow版本: {tf.__version__}")
print(f"是否内置CUDA支持: {tf.test.is_built_with_cuda()}")

# 2. 检测物理GPU
gpus = tf.config.list_physical_devices('GPU')
print(f"\n检测到的GPU数量: {len(gpus)}")
if gpus:
    for i, gpu in enumerate(gpus):
        print(f"GPU {i}: {gpu}")
    print("✅ 已启用GPU支持！")
else:
    print("❌ 未检测到GPU，仅使用CPU运行")

# 3. 测试GPU计算
if gpus:
    with tf.device('/GPU:0'):
        a = tf.constant([1.0, 2.0, 3.0])
        b = tf.constant([4.0, 5.0, 6.0])
        c = a + b
        print(f"\nGPU计算结果: {c.numpy()}")
        print(f"计算设备: {c.device}")