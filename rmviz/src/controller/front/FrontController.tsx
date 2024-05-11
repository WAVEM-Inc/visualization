import React from 'react';
import { Route, Switch } from 'react-router-dom';
import EntryPage from '../../page/entry/EntryPage';

const FrontController: React.FC = (): React.ReactElement<any, any> | null => {

    return (
        <Switch>
            <Route exact path={"/"}>
                <EntryPage />
            </Route>
        </Switch>
    );
}

export default FrontController;