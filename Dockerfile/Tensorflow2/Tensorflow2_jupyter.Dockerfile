FROM tensorflow/tensorflow:2.7.0-gpu-jupyter
ARG USERNAME=park

RUN useradd --user-group --system --create-home --no-log-init ${USERNAME} && \
    usermod -aG sudo ${USERNAME}

RUN jupyter nbextension enable --py widgetsnbextension

USER ${USERNAME}
WORKDIR /home/${USERNAME}

CMD ["jupyter", "notebook", "--ip", "0.0.0.0", "--no-browser", "--allow-root"]
