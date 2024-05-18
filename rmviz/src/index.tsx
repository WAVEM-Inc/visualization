import ReactDOM from 'react-dom/client';
import { BrowserRouter } from "react-router-dom";
import App from './App';
import './index.css';
import reportWebVitals from './reportWebVitals';

const root = ReactDOM.createRoot(
  document.getElementById('root') as HTMLElement
);

declare global {
  interface Window {
    google: typeof google.maps;
  }
}

root.render(
  <BrowserRouter>
    <App />
  </BrowserRouter>
);

reportWebVitals();