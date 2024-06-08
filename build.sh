#!/bin/bash

# Create a directory for the distribution
mkdir -p dist

# Copy the header file into the distribution directory
cp graphkit.h dist/

# Create a compressed archive
tar -czvf graphkit.tar.gz dist/

# Clean up temporary files
rm -rf dist
