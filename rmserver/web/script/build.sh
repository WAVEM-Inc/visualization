#!/bin/bash
npm run server_build
npm run client_build

if ! grep -qxF 'alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/web && npm run launch"' ~/.bashrc; then
    echo 'alias rmviz="cd ~/kec_ws/src/service/rmviz/rmserver/web && npm run launch"' >> ~/.bashrc
fi

source ~/.bashrc
