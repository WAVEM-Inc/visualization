import './App.css';
import DataController from './controller/data/DataController';
import FrontController from './controller/front/FrontController';

function App() {

  return (
    <div className="App">
      <FrontController />
      <DataController />
    </div>
  );
}

export default App;
