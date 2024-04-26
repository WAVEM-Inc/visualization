import { Route, Switch } from 'react-router-dom';
import './App.css';
import BlueSpaceDashBoardPage from './page/bluespace/BlueSpaceDashBoardPage';
import DashBoardPage from './page/dashboard/DashBoardPage';
import DataBoardPage from './page/databoard/DataBoardPage';
import PathEditPage from "./page/pathEdit/PathEditPage";
import TestPage from './page/test/TestPage';

function App() {
  return (
    <div className="App">
      <Switch>
        <Route exact path={"/"}>
          <DashBoardPage />
        </Route>
        <Route exact path={"/bluespace"}>
          <BlueSpaceDashBoardPage />
        </Route>
        <Route exact path={"/data"}>
          <DataBoardPage />
        </Route>
        <Route exact path={"/path/edit"}>
          <PathEditPage />
        </Route>
        <Route exact path={"/test"}>
          <TestPage />
        </Route>
      </Switch>
    </div>
  );
}

export default App;
