import { Route, Switch } from 'react-router-dom';
import './App.css';
import BlueSpaceDashBoardPage from './page/bluespace/BlueSpaceDashBoardPage';
import DataBoardPage from './page/databoard/DataBoardPage';
import KECDashBoardPage from './page/kec/KECDashBoardPage';
import DashBoardPage from './page/dashboard/DashBoardPage';

function App() {
  return (
    <div className="App">
      <Switch>
        <Route exact path={"/"}>
          <DashBoardPage />
        </Route>
        <Route exact path={"/kec"}>
          <KECDashBoardPage />
        </Route>
        <Route exact path={"/bluespace"}>
          <BlueSpaceDashBoardPage />
        </Route>
        <Route exact path={"/data"}>
          <DataBoardPage />
        </Route>
      </Switch>
    </div>
  );
}

export default App;
