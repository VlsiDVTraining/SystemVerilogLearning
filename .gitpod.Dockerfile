FROM gitpod/workspace-full
RUN sudo apt-get update
RUN sudo apt-get install -y verilator
RUN sudo apt-get install -y iverilog
RUN sudo apt-get install -y gtkwave
RUN sudo apt-get install -y yosys
