#ifndef SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H
#define SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H

#include <iostream>
#include <cstdio>
#include <unistd.h>

namespace sonar_3d_reconstruction
{

// RAII helper to suppress stdout/stderr at file descriptor level.
// Used to silence verbose third-party (OctoMap) output during operations.
class SuppressOutput {
public:
    SuppressOutput() {
        std::cout.flush();
        std::cerr.flush();
        fflush(stdout);
        fflush(stderr);
        stdout_fd_ = dup(fileno(stdout));
        stderr_fd_ = dup(fileno(stderr));
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
    }

    ~SuppressOutput() {
        fflush(stdout);
        fflush(stderr);
        dup2(stdout_fd_, fileno(stdout));
        dup2(stderr_fd_, fileno(stderr));
        close(stdout_fd_);
        close(stderr_fd_);
    }

    // Non-copyable, non-movable (manages OS resources)
    SuppressOutput(const SuppressOutput&) = delete;
    SuppressOutput& operator=(const SuppressOutput&) = delete;
    SuppressOutput(SuppressOutput&&) = delete;
    SuppressOutput& operator=(SuppressOutput&&) = delete;

private:
    int stdout_fd_;
    int stderr_fd_;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H
