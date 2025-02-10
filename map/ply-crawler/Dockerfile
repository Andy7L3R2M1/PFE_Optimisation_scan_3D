FROM python:3.8-bookworm

USER root

# Install necessary packages
RUN apt update && apt upgrade -y && \
    # library needed by pymeshlab
    apt install -y libglu1-mesa-dev && \
    apt clean

# set up directories
RUN mkdir -p /home/app

# set working directory
WORKDIR /home/app

# Create a virtual environment and install dependencies
RUN python3 -m venv venv
ENV PATH="/home/app/venv/bin:$PATH"
COPY requirements.txt requirements.txt
RUN pip install --upgrade pip && pip install -r requirements.txt

# copy the rest of the application code
COPY . .

# run the application
CMD ["python", "-u", "app.py"]