FROM gitpod/workspace-full

# Verilator dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    git perl python3 make autoconf g++ flex bison ccache libfl2 libfl-dev zlib1g zlib1g-dev

# Install the latest Verilator from source
RUN git clone https://github.com/verilator/verilator.git && \
    cd verilator && \
    git checkout master && \
    autoconf && \
    ./configure && \
    make -j$(nproc) && \
    sudo make install && \
    cd .. && rm -rf verilator

# Install Icarus Verilog (iverilog) from the package manager (latest apt repo version)
RUN sudo apt-get install -y iverilog

# (Optional) Install GTKWave for viewing waveforms
RUN sudo apt-get install -y gtkwave
