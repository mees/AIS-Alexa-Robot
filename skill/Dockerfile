FROM python:3.6
WORKDIR /skill
COPY deps deps
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt
COPY ais_alexa_skill.py ais_alexa_skill.py

EXPOSE 80

CMD ["python", "ais_alexa_skill.py"]
