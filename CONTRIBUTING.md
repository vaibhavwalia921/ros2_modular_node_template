# Contributing to ROS 2 Modular Node Template

Thank you for your interest in contributing to this project! This document provides guidelines and instructions for contributing.

## Code of Conduct

This project adheres to a code of conduct. By participating, you are expected to uphold this code.

## How to Contribute

### Reporting Issues

- Check if the issue already exists
- Provide a clear description of the problem
- Include steps to reproduce
- Mention your environment (ROS version, OS, etc.)

### Submitting Changes

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Ensure all tests pass (`colcon test`)
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

### Coding Standards

#### C++ Style Guide

- Follow the [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
- Use `clang-format` with the provided `.clang-format` file
- Naming conventions:
  - Classes: `PascalCase`
  - Functions/Methods: `snake_case`
  - Variables: `snake_case`
  - Constants: `UPPER_SNAKE_CASE`
  - Namespaces: `snake_case`

#### Python Style Guide

- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- Use `black` for formatting
- Use type hints where appropriate

### Testing

- Write unit tests for new functionality
- Ensure code coverage remains above 80%
- Integration tests for ROS interfaces
- Performance tests for critical paths

### Documentation

- Update README.md if needed
- Add inline documentation for complex logic
- Update architecture docs for structural changes
- Include examples for new features

## Development Setup

### Prerequisites

```bash
# Install development tools
sudo apt-get update
sudo apt-get install -y \
    clang-format \
    clang-tidy \
    python3-black \
    python3-pytest \
    lcov

# Install pre-commit hooks
pip3 install pre-commit
pre-commit install