# Use Ubuntu as the base image
FROM ubuntu:20.04

# Install necessary packages
RUN apt-get update && apt-get install -y \
    gcc \
    make \
    socat \
    build-essential \
    net-tools

# Copy the whole project to the container
COPY . /usr/src/app

# Set working directory
WORKDIR /usr/src/app

# Build the project using Makefile
RUN make

# Expose necessary ports (optional, for debugging purposes)
EXPOSE 9600

# Run the compiled binary for the cable program by default
CMD ["./bin/cable"]
