# Use Ubuntu as the base image
FROM ubuntu:20.04

# Install necessary packages
RUN apt-get update && apt-get install -y \
    gcc \
    make \
    socat \
    build-essential

# Copy the whole project to the container
COPY . /usr/src/app

# Set working directory
WORKDIR /usr/src/app

# Build the project using Makefile
RUN make

# Run the compiled binary for the cable program
CMD ["./bin/cable"]
