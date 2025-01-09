# 1. Workspace

<pre><code>mkdir -p ~/flow_ws/src
cd ~/flow_ws/src
git clone https://github.com/hwangjuntae/isaacsim_crowd_simulation
</code></pre>

**edit pke name \<isaacsim_crowd_simulation\> to \<moving_people\>**


# 2. install dependencies pkgs

## install pegasus simulator

<pre><code>
clone git 

git clone https://github.com/PegasusSimulator/PegasusSimulator.git
</code></pre>


refer this page, applicate on isaac sim


[install pegasus](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-the-pegasus-simulator)

## install psf(py social force)
<pre><code>
conda create -n PySocialForce python=3.10 -y
echo 'alias psf="conda activate PySocialForce"' >> ~/.bashrc
source ~/.bashrc
psf
sudo apt update -y
sudo apt install python3-urllib3 -y
pip install psutil pyyaml
pip install pysocialforce[test,plot]
sudo apt install -y python3 python3-pip
sudo apt install -y imagemagick
pip install numpy matplotlib
</code></pre>


psf execute

<pre><code>
psf
cd ~/flow_ws/src/moving_people/src/py_social_force/my_psf.py
python3 my_psf.py
</code></pre>
