```
clang++ main.cpp -o my_vulkan_app \                                [3b9d032] 
    -std=c++17 \
    -O2 \
    -I$VULKAN_SDK/include \
    -I/usr/local/include \
    -L$VULKAN_SDK/lib \
    -L/usr/local/lib \
    -lglfw \
    -lvulkan \
    -framework Cocoa \
    -framework IOKit \
    -framework CoreVideo \
    -lm

```

```
source ../../../VulkanSDK/1.3.296.0/setup-env.sh 
```
