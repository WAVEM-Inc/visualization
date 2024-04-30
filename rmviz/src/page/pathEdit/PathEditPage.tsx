import JSONInput from 'react-json-editor-ajrm';
import * as jso from "../../assets/json/bluespace/1F/201-202.json";
import TopComponents from '../../components/top/TopComponent';
import "./PathEditPage.css";

const lo_en: any = require("react-json-editor-ajrm/locale/en");

const PathEditPage: React.FC = () => {
    return (
        <div className="path_edit_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="json_edit_conatiner">
                <JSONInput
                    id="json_text_area"
                    placeholder={jso}
                    theme="dark"
                    locale={lo_en}
                    colors={{
                        string: "#DAA520"
                    }}
                    height="550px"
                />
            </div>
        </div>
    );
}

export default PathEditPage;