# Automated Infinite Recharge #
## Team SHARP 3260 ##

<iframe width="560" height="315" src="https://www.youtube.com/embed/mh1IuxmVZco" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Python client written by FRC Team 3260 for the [FRC 2020 simulator](https://github.com/ptkinvent/frcsim2020). Uses
simulated LIDAR and model of our 2020 robot to play Infinite Recharge 100% autonomously.

Set up and launch:
```sh
pip3 install -r requirements.txt # Installs all dependencies from requirements.txt
python3 main.py                  # Sim must be running and transmitting LIDAR points
```

Tests:
```sh
python3 -m unittest -v
# Or
pytest tests
```

Generate and view documentation (also available at automatedinfiniterecharge.readthedocs.io):
```sh
cd docs/
make html
firefox _build/html/index.html
```
