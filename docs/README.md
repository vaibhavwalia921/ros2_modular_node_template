# Documentation

This directory contains comprehensive documentation for the ROS 2 Modular Node Template project.

## Documentation Structure

### [Architecture Documentation](architecture.md)
Detailed architectural information including:
- High-level system design and layer separation
- Data flow diagrams and component interactions
- State machine design and transitions
- Communication patterns and threading model
- Performance considerations and deployment strategies
- Extensibility guidelines

### [API Documentation](api_documentation.md)
Complete API reference including:
- Core layer APIs (StateMachine, IAlgorithm, Data Types)
- Middleware layer APIs (LifecycleNodeWrapper, RosInterfaces, ParameterHandler)
- Algorithm layer APIs (ExampleAlgorithm and extension guidelines)
- Error handling patterns and best practices
- Code examples and usage patterns

### [Diagrams](diagrams/)
Visual representations of the system architecture:
- [State Machine Diagram](diagrams/state_machine.puml) - PlantUML diagram showing operational state transitions

## Quick Reference

### For Users
If you're using the template as a foundation for your own nodes:
1. Start with the main [README.md](../README.md) for setup and usage instructions
2. Review the [Architecture Documentation](architecture.md) to understand the design principles
3. Use the [API Documentation](api_documentation.md) as a reference for available interfaces

### For Developers
If you're extending or modifying the template:
1. Read the [Architecture Documentation](architecture.md) thoroughly
2. Refer to the [API Documentation](api_documentation.md) for implementation details
3. Check the [State Machine Diagram](diagrams/state_machine.puml) for state transition rules

### For Contributors
If you're contributing to the template itself:
1. Understand the architectural principles in [Architecture Documentation](architecture.md)
2. Follow the API patterns documented in [API Documentation](api_documentation.md)
3. Ensure all changes maintain backward compatibility where possible

## Documentation Standards

### Code Documentation
- All public APIs are documented with Doxygen-style comments
- Header files contain comprehensive interface documentation
- Implementation files include inline comments for complex logic

### Architecture Documentation
- Uses Markdown format for easy reading and editing
- Includes diagrams for visual understanding
- Provides both high-level and detailed views
- Explains design decisions and trade-offs

### API Documentation
- Complete method signatures and parameter descriptions
- Usage examples for common scenarios
- Error handling patterns and best practices
- Cross-references between related components

## Contributing to Documentation

When contributing to the project, please ensure:

1. **Update API Documentation**: Add documentation for any new public APIs
2. **Update Architecture Documentation**: Document any architectural changes
3. **Update Diagrams**: Keep visual representations in sync with code
4. **Examples**: Provide clear examples for new features
5. **Consistency**: Follow existing documentation style and format

## Generating Documentation

### Doxygen (Future Enhancement)
The project is structured to support Doxygen documentation generation. To generate HTML documentation:

```bash
# Install Doxygen
sudo apt-get install doxygen

# Generate documentation (Doxyfile to be added)
doxygen Doxyfile
```

### PlantUML Diagrams
Diagrams are created using PlantUML. To render diagrams:

```bash
# Install PlantUML
sudo apt-get install plantuml

# Render diagram to PNG
plantuml -tpng docs/diagrams/state_machine.puml
```

## Documentation Coverage

The documentation aims to cover:
- ✅ 100% of public APIs
- ✅ All architectural components
- ✅ State machine transitions
- ✅ Communication patterns
- ✅ Extension points and guidelines
- ⏳ Performance benchmarks (future enhancement)
- ⏳ Migration guides (future enhancement)

## Feedback and Improvements

If you find gaps in the documentation or have suggestions for improvement:

1. Check the [issue tracker](https://github.com/your-repo/issues) for existing documentation issues
2. Create a new issue if needed
3. Consider submitting a pull request with documentation improvements

## Related Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Lifecycle Documentation](https://design.ros2.org/articles/node_lifecycle.html)
- [C++ Best Practices](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines.html)
- [Doxygen Documentation](https://www.doxygen.nl/index.html)
- [PlantUML Documentation](https://plantuml.com/)