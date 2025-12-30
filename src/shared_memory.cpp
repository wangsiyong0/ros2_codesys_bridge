#include "ros2_codesys_bridge/shared_memory.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

namespace ros2_codesys_bridge
{
    SharedMemory::SharedMemory(const std::string &name, size_t size)
        : shm_fd_(-1), ptr_(nullptr), data_(nullptr), valid_(false), name_(name)
    {
        // 创建共享内存
        shm_fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1)
        {
            std::cerr << "Failed to open shared memory" << std::endl;
            return;
        }

        // 设置共享内存大小
        if (ftruncate(shm_fd_, size) == -1)
        {
            std::cerr << "Failed to set shared memory size" << std::endl;
            close(shm_fd_);
            return;
        }

        // 映射共享内存
        ptr_ = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (ptr_ == MAP_FAILED)
        {
            std::cerr << "Failed to map shared memory" << std::endl;
            close(shm_fd_);
            return;
        }

        data_ = static_cast<SharedMemoryData*>(ptr_);
        valid_ = true;
    }

    SharedMemory::~SharedMemory()
    {
        if (ptr_ != MAP_FAILED)
            munmap(ptr_, sizeof(SharedMemoryData));
        if (shm_fd_ != -1)
            close(shm_fd_);
        shm_unlink(name_.c_str());
    }

    void SharedMemory::lock()
    {
        // 实际应用中应使用信号量进行同步
    }

    void SharedMemory::unlock()
    {
        // 实际应用中应使用信号量进行同步
    }
}  // namespace ros2_codesys_bridge