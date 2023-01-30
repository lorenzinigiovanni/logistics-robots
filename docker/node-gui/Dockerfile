FROM node:18

RUN mkdir -p /usr/app/node_modules
WORKDIR /usr/app

COPY ./GUI/package*.json ./
RUN npm install --quiet

EXPOSE 4200
