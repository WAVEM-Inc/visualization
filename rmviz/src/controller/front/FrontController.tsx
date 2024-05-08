import React from 'react';
import { Route, Switch } from 'react-router-dom';
import DashBoardPage from '../../page/dashboard/DashBoardPage';

const FrontController: React.FC = (): React.ReactElement<any, any> | null => {

    return (
        <Switch>
            <Route exact path={"/"}>
                <DashBoardPage />
            </Route>
        </Switch>
    );
}

export default FrontController;