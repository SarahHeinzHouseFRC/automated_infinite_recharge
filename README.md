# Autonomous Infinite Recharge #
## Team SHARP 3260 ##

[![Watch the video](https://img.youtube.com/vi/mh1IuxmVZco/maxresdefault.jpg)](https://youtu.be/mh1IuxmVZco)

Python client written by FRC Team 3260 designed to work with the [FRC 2020
simulator](https://github.com/SarahHeinzHouseFRC/frcsim2020). Adds artificial intelligence to the simulated LIDAR and
virtual model of our 2020 robot in the sim to play Infinite Recharge with zero human intervention.

## Setup ##

You will first need to build and run the [FRC 2020 simulator](https://github.com/SarahHeinzHouseFRC/frcsim2020). Ensure
at least one robot in the sim is configured in the YAML config file to transmit LIDAR points. Then:

```sh
pip3 install -r requirements.txt
python3 main.py # Optionally, specify player number with --player
```

## Tests ##

```sh
python3 -m unittest -v
# Or
pytest tests
```

## Docs ##

Generate and view documentation (also available at [ReadTheDocs](https://automatedinfiniterecharge.readthedocs.io)):
```sh
cd docs/
make html
firefox _build/html/index.html
```
