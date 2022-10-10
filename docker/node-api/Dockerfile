FROM node:16

RUN mkdir -p /usr/app/node_modules
WORKDIR /usr/app

COPY ./API/package*.json ./
RUN npm install --quiet

EXPOSE 3000
