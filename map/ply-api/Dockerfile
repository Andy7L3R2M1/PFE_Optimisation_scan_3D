FROM python:3.9.2-alpine

# upgrade pip and install curl for healthchecks
RUN apk add --no-cache curl && \
    pip install --upgrade pip

# set up directories
RUN mkdir -p /home/app /var/log/flask-app && \
    touch /var/log/flask-app/flask-app.err.log /var/log/flask-app/flask-app.out.log

# set working directory
WORKDIR /home/app

# copy and install dependencies before application code for better layer caching
COPY requirements.txt requirements.txt
RUN python -m venv venv && \
    . venv/bin/activate && \
    pip install -r requirements.txt

# copy the rest of the application code
COPY . .

# set environment variables
ENV VIRTUAL_ENV=/home/app/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV FLASK_APP=app.py

# run the application
CMD ["python", "app.py"]