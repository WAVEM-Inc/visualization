import React from 'react';
import { Route, Switch } from 'react-router-dom';
import DashBoardPage from '../../page/dashboard/DashBoardPage';
import DataBoardPage from '../../page/databoard/DataBoardPage';
import PathEditPage from '../../page/pathEdit/PathEditPage';

const FrontController: React.FC = (): React.ReactElement<any, any> | null => {

    return (
        <Switch>
            <Route exact path={"/"}>
                <DashBoardPage />
            </Route>
            <Route exact path={"/data"}>
                <DataBoardPage />
            </Route>
            <Route exact path={"/path/edit"}>
                <PathEditPage />
            </Route>
        </Switch>
    );
}

export default FrontController;