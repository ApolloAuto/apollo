/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtEnginio module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef ENGINIOMODELBASE_P_H
#define ENGINIOMODELBASE_P_H


#include <Enginio/enginiomodel.h>
#include <Enginio/enginioreply.h>
#include <Enginio/private/enginioclient_p.h>
#include <Enginio/private/enginiofakereply_p.h>
#include <Enginio/private/enginiodummyreply_p.h>
#include <Enginio/enginioreplystate.h>
#include <Enginio/private/enginiobackendconnection_p.h>
#include <Enginio/enginiobasemodel.h>
#include <Enginio/private/enginiobasemodel_p.h>

#include <QtCore/qdatetime.h>
#include <QtCore/qdebug.h>
#include <QtCore/qhash.h>
#include <QtCore/qjsonobject.h>
#include <QtCore/qjsonarray.h>
#include <QtCore/qstring.h>
#include <QtCore/quuid.h>
#include <QtCore/qvector.h>

#include <QtCore/private/qabstractitemmodel_p.h>

QT_BEGIN_NAMESPACE

enum {
    DeletedRow = -3,
    NoHintRow = -4,
    InvalidRow = NoHintRow
};

struct EnginioModelPrivateAttachedData
{
    uint ref;
    int row;
    QString id;
    EnginioReplyState *createReply;
    EnginioModelPrivateAttachedData(int initRow = DeletedRow, const QString &initId = QString())
        : ref()
        , row(initRow)
        , id(initId)
        , createReply()
    {}
};
Q_DECLARE_TYPEINFO(EnginioModelPrivateAttachedData, Q_MOVABLE_TYPE);

#ifndef QT_NO_DEBUG_STREAM
QDebug operator<<(QDebug dbg, const EnginioModelPrivateAttachedData &a);
#endif

class AttachedDataContainer
{
    typedef int Row;
    typedef int StorageIndex;
    typedef QString ObjectId;
    typedef QString RequestId;
    typedef EnginioModelPrivateAttachedData AttachedData;

    typedef QHash<Row, StorageIndex> RowIndex;
    RowIndex _rowIndex;

    typedef QHash<ObjectId, StorageIndex> ObjectIdIndex;
    ObjectIdIndex _objectIdIndex;

    typedef QHash<RequestId, QPair<int /*ref*/, StorageIndex> > RequestIdIndex;
    RequestIdIndex _requestIdIndex;

    typedef QHash<StorageIndex, AttachedData> Storage;
    QVector<AttachedData> _storage; // TODO replace by something smarter so we can use pointers instead of index.

    enum { InvalidStorageIndex = InvalidRow };

    StorageIndex append(const AttachedData &data)
    {
        _storage.append(data);
        StorageIndex idx = _storage.count() - 1;
        _rowIndex.insert(data.row, idx);
        _objectIdIndex.insert(data.id, idx);
        return idx;
    }

public:
    bool contains(const ObjectId &id) const
    {
        return _objectIdIndex.contains(id);
    }

    Row rowFromObjectId(const ObjectId &id) const
    {
        Q_ASSERT(contains(id));
        StorageIndex idx = _objectIdIndex.value(id, InvalidStorageIndex);
        return idx == InvalidStorageIndex ? InvalidRow : _storage[idx].row;
    }

    Row rowFromRequestId(const RequestId &id) const
    {
        StorageIndex idx = _requestIdIndex.value(id, qMakePair(0, static_cast<int>(InvalidStorageIndex))).second;
        return idx == InvalidStorageIndex ? InvalidRow : _storage[idx].row;
    }

    bool isSynced(Row row) const
    {
        return _storage[_rowIndex.value(row)].ref == 0;
    }

    void updateAllDataAfterRowRemoval(const int row) {
        _rowIndex.clear();
        _rowIndex.reserve(_storage.count());
        for (StorageIndex i = 0; i < _storage.count() ; ++i) {
            AttachedData &data = _storage[i];
            if (data.row > row)
                --data.row;
            else if (data.row == row)
                data.row = DeletedRow;
            _rowIndex.insert(data.row, i);
        }
    }

    AttachedData &ref(const ObjectId &id, Row row)
    {
        StorageIndex idx = _objectIdIndex.value(id, InvalidStorageIndex);
        if (idx == InvalidStorageIndex) {
            AttachedData data(row, id);
            idx = append(data);
        }
        AttachedData &data = _storage[idx];
        ++data.ref;
        Q_ASSERT(_storage[idx].ref == 1 || _storage[idx].row == row);
        data.row = row;
        return data;
    }

    AttachedData &ref(Row row)
    {
        StorageIndex idx = _rowIndex.value(row, InvalidStorageIndex);
        Q_ASSERT(idx != InvalidStorageIndex);
        AttachedData &data = _storage[idx];
        ++data.ref;
        return data;
    }

    AttachedData &deref(const ObjectId &id)
    {
        StorageIndex idx = _objectIdIndex.value(id, InvalidStorageIndex);
        Q_ASSERT(idx != InvalidStorageIndex);
        AttachedData &attachedData = _storage[idx];
        if (!--attachedData.ref && id[0] == 't') {
            // TODO it is last ref to a tmp id we should remove it
        }
        return attachedData;
    }

    void insert(const AttachedData &data)
    {
        _storage.append(data);
        StorageIndex idx = _storage.count() - 1;
        _rowIndex.insert(data.row, idx);
        _objectIdIndex.insert(data.id, idx);
    }

    void insertRequestId(const RequestId &id, Row row)
    {
        StorageIndex idx = _rowIndex.value(row, InvalidStorageIndex);
        Q_ASSERT(idx != InvalidStorageIndex);
        _requestIdIndex.insert(id, qMakePair(2, idx));
    }

    /*!
      \internal
      returns true if the request was already handled
    */
    bool markRequestIdAsHandled(const RequestId &id)
    {
        RequestIdIndex::iterator::reference value = _requestIdIndex[id];
        if (value.first) {
            if (--value.first <= 0) {
                _requestIdIndex.remove(id);
                return true;
            }
        } else {
            _requestIdIndex.remove(id);
        }
        return false;
    }

    void initFromArray(const QJsonArray &array)
    {
        const int count = array.count();
        _storage.clear();
        _rowIndex.clear();
        _objectIdIndex.clear();

        _storage.reserve(count);
        _rowIndex.reserve(count);
        _objectIdIndex.reserve(count);

        for (int row = 0; row < count; ++row) {
            QString id = array[row].toObject()[EnginioString::id].toString();
            Q_ASSERT(!id.isEmpty());
            AttachedData data(row, id);
            _storage.append(data);
            _rowIndex.insert(row, row);
            _objectIdIndex.insert(id, row);
        }
    }
};


class ENGINIOCLIENT_EXPORT EnginioBaseModelPrivate : public QAbstractItemModelPrivate {
protected:
    EnginioClientConnectionPrivate *_enginio;
    Enginio::Operation _operation;
    EnginioBaseModel *q;
    QVector<QMetaObject::Connection> _clientConnections;
    QObject *_replyConnectionConntext;

    const static int IncrementalModelUpdate;
    typedef EnginioModelPrivateAttachedData AttachedData;
    AttachedDataContainer _attachedData;
    int _latestRequestedOffset;
    bool _canFetchMore;

    unsigned _rolesCounter;
    QHash<int, QString> _roles;

    QJsonArray _data;

    class NotificationObject {
        // connection object it can be:
        // - null if not yet created
        // - -1 if notifications where disabled with EnginioModel::disableNotifications()
        // - valid pointer to connection object
        EnginioBackendConnection *_connection;

        struct NotificationReceived
        {
            EnginioBaseModelPrivate *model;

            void operator ()(QJsonObject data)
            {
                model->receivedNotification(data);
            }
        };
        void removeConnection()
        {
            if (*this) {
                _connection->close();
                delete _connection;
            }
        }

    public:
        NotificationObject()
            : _connection()
        {}

        ~NotificationObject()
        {
            removeConnection();
        }

        operator EnginioBackendConnection*()
        {
            return qintptr(_connection) != -1 ? _connection : 0;
        }

        void disable()
        {
            removeConnection();
            _connection = (EnginioBackendConnection*)-1;
        }

        void connectToBackend(EnginioBaseModelPrivate *model, EnginioClientConnectionPrivate *enginio, const QJsonObject &filter)
        {
            if (qintptr(_connection) == -1)
                return;
            Q_ASSERT(model && enginio);
            if (enginio->_serviceUrl != EnginioString::stagingEnginIo)
                return;  // TODO it allows to use notification only on staging
            removeConnection(); // TODO reuse the connecton object
            _connection = new EnginioBackendConnection;
            NotificationReceived receiver = { model };
            QObject::connect(_connection, &EnginioBackendConnection::dataReceived, receiver);
            _connection->connectToBackend(enginio, filter);
        }
    } _notifications;

    struct FinishedRemoveRequest
    {
        EnginioBaseModelPrivate *model;
        const QString id;
        EnginioReplyState *reply;
        void operator ()()
        {
            model->finishedRemoveRequest(reply, id);
        }
    };

    struct FinishedUpdateRequest
    {
        EnginioBaseModelPrivate *model;
        const QString id;
        const QJsonObject oldValue;
        EnginioReplyState *reply;
        void operator ()()
        {
            model->finishedUpdateRequest(reply, id, oldValue);
        }
    };

    struct FinishedCreateRequest
    {
        EnginioBaseModelPrivate *model;
        const QString tmpId;
        EnginioReplyState *reply;
        void operator ()()
        {
            model->finishedCreateRequest(reply, tmpId);
        }
    };

    struct FinishedFullQueryRequest
    {
        EnginioBaseModelPrivate *model;
        EnginioReplyState *reply;
        void operator ()()
        {
            model->finishedFullQueryRequest(reply);
        }
    };

    struct FinishedIncrementalUpdateRequest
    {
        EnginioBaseModelPrivate *model;
        const QJsonObject query;
        EnginioReplyState *reply;
        void operator ()()
        {
            model->finishedIncrementalUpdateRequest(reply, query);
        }
    };

    class QueryChanged
    {
        EnginioBaseModelPrivate *model;
    public:
        QueryChanged(EnginioBaseModelPrivate *m)
            : model(m)
        {
            Q_ASSERT(m);
        }

        void operator ()()
        {
            model->execute();
        }
    };

    class RefreshQueryAfterAuthChange // It is needed for compilers that don't support variadic templates
    {
        EnginioBaseModelPrivate *model;
    public:
        RefreshQueryAfterAuthChange(EnginioBaseModelPrivate *m)
            : model(m)
        {
            Q_ASSERT(m);
        }

        void operator ()(Enginio::AuthenticationState state) const
        {
            // TODO we do not want to refresh on a failed attempt to login
            if (state == Enginio::NotAuthenticated // logout
                || state == Enginio::Authenticated // successful login
                || state == Enginio::AuthenticationFailure)  // token refresh failed
                    model->execute();
        }
    };

public:
    EnginioBaseModelPrivate(EnginioBaseModel *q_ptr)
        : _enginio(0)
        , _operation()
        , q(q_ptr)
        , _replyConnectionConntext(new QObject())
        , _latestRequestedOffset(0)
        , _canFetchMore(false)
        , _rolesCounter(Enginio::SyncedRole)
    {
    }

    virtual ~EnginioBaseModelPrivate();

    void disableNotifications()
    {
        _notifications.disable();
    }

    void receivedNotification(const QJsonObject &data);
    void receivedRemoveNotification(const QJsonObject &object, int rowHint = NoHintRow);
    void receivedUpdateNotification(const QJsonObject &object, const QString &idHint = QString(), int row = NoHintRow);
    void receivedCreateNotification(const QJsonObject &object);

    EnginioReplyState *append(const QJsonObject &value)
    {
        QJsonObject object(value);
        QString temporaryId = QString::fromLatin1("tmp") + QUuid::createUuid().toString();
        object[EnginioString::objectType] = queryData(EnginioString::objectType); // TODO think about it, it means that not all queries are valid
        ObjectAdaptor<QJsonObject> aObject(object);
        QNetworkReply *nreply = _enginio->create(aObject, _operation);
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        FinishedCreateRequest finishedRequest = { this, temporaryId, ereply };
        QObject::connect(ereply, &EnginioReplyState::dataChanged, _replyConnectionConntext, finishedRequest);
        object[EnginioString::id] = temporaryId;
        const int row = _data.count();
        AttachedData data(row, temporaryId);
        data.ref = 1;
        data.createReply = ereply;
        if (!row) { // the first item need to update roles
            q->beginResetModel();
            _attachedData.insert(data);
            _data.append(value);
            syncRoles();
            q->endResetModel();
        } else {
            q->beginInsertRows(QModelIndex(), _data.count(), _data.count());
            _attachedData.insert(data);
            _data.append(value);
            q->endInsertRows();
        }
        _attachedData.insertRequestId(ereply->requestId(), row);
        return ereply;
    }

    struct SwapNetworkReplyBase
    {
        EnginioReplyState *_reply;
        EnginioBaseModelPrivate *_model;
        QJsonObject _object;
        QString _tmpId;
        QPointer<EnginioBaseModel> _modelGuard;

        void markAsError(QByteArray msg)
        {
            EnginioFakeReply *nreply = new EnginioFakeReply(_reply, EnginioClientConnectionPrivate::constructErrorMessage(msg));
            _reply->setNetworkReply(nreply);
        }

        QPair<QString, int> getAndSetCurrentIdRow(EnginioReplyState *finishedCreateReply)
        {
            QString id = _model->replyData(finishedCreateReply)[EnginioString::id].toString();
            Q_ASSERT(!id.isEmpty());
            _object[EnginioString::id] = id;
            int row = InvalidRow;
            if (Q_LIKELY(_model->_attachedData.contains(_tmpId)))
                row = _model->_attachedData.deref(_tmpId).row;
            else if (Q_LIKELY(_model->_attachedData.contains(id))) {
                // model reset happend in a mean while
                row = _model->_attachedData.rowFromObjectId(id);
            } else {
                // the model was reset, probably with a different query, beacause
                // we have no sign of the id.
            }
            return qMakePair(id, row);
        }

        void swapNetworkReply(EnginioReplyState *ereply)
        {
            _reply->swapNetworkReply(ereply);
            ereply->deleteLater();
        }
    };

    struct SwapNetworkReplyForRemove
    {
        SwapNetworkReplyBase d;
        EnginioReplyState *finishedCreateReply;
        void operator ()()
        {
            if (finishedCreateReply->isError()) {
                d.markAsError(EnginioString::Dependent_create_query_failed_so_object_could_not_be_removed);
            } else if (Q_UNLIKELY(!d._modelGuard)) {
                d.markAsError(EnginioString::EnginioModel_was_removed_before_this_request_was_prepared);
            } else {
                QPair<QString, int> tmp = d.getAndSetCurrentIdRow(finishedCreateReply);
                const int row = tmp.second;
                if (Q_UNLIKELY(row == InvalidRow)) {
                    d.markAsError(EnginioString::EnginioModel_The_query_was_changed_before_the_request_could_be_sent);
                    return;
                }
                QString id = tmp.first;
                FinishedRemoveRequest finishedRequest = { d._model, id, d._reply };
                QObject::connect(d._reply, &EnginioReplyState::dataChanged, d._model->_replyConnectionConntext, finishedRequest);
                EnginioReplyState *ereply = d._model->removeNow(row, d._object, id);
                d.swapNetworkReply(ereply);
            }
        }
    };

    EnginioReplyState *remove(int row)
    {
        QJsonObject oldObject = _data.at(row).toObject();
        QString id = oldObject[EnginioString::id].toString();
        if (id.isEmpty())
            return removeDelayed(row, oldObject);
        return removeNow(row, oldObject, id);
    }

    EnginioReplyState *removeDelayed(int row, const QJsonObject &oldObject)
    {
        // We are about to remove a not synced new item. The item do not have id yet,
        // so we can not make a request now, we need to wait for finished signal.
        EnginioReplyState *ereply, *createReply;
        QString tmpId;
        Q_ASSERT(oldObject[EnginioString::id].toString().isEmpty());
        delayedOperation(row, &ereply, &tmpId, &createReply);
        SwapNetworkReplyForRemove swapNetworkReply = {{ereply, this, oldObject, tmpId, q}, createReply};
        QObject::connect(createReply, &EnginioReplyState::dataChanged, swapNetworkReply);
        return ereply;
    }

    EnginioReplyState *removeNow(int row, const QJsonObject &oldObject, const QString &id)
    {
        Q_ASSERT(!id.isEmpty());
        _attachedData.ref(id, row); // TODO if refcount is > 1 then do not emit dataChanged
        ObjectAdaptor<QJsonObject> aOldObject(oldObject);
        QNetworkReply *nreply = _enginio->remove(aOldObject, _operation);
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        FinishedRemoveRequest finishedRequest = { this, id, ereply };
        QObject::connect(ereply, &EnginioReplyState::dataChanged, _replyConnectionConntext, finishedRequest);
        _attachedData.insertRequestId(ereply->requestId(), row);
        QVector<int> roles(1);
        roles.append(Enginio::SyncedRole);
        emit q->dataChanged(q->index(row), q->index(row) , roles);
        return ereply;
    }

    EnginioReplyState *setValue(int row, const QString &role, const QVariant &value)
    {
        int key = _roles.key(role, Enginio::InvalidRole);
        return setData(row, value, key);
    }

    Enginio::Operation operation() const Q_REQUIRED_RESULT
    {
        return _operation;
    }

    void setOperation(const int operation)
    {
        Q_ASSERT_X(operation >= Enginio::ObjectOperation, "setOperation", "Invalid operation specified.");
        _operation = static_cast<Enginio::Operation>(operation);
    }

    void execute()
    {
        if (!_enginio || _enginio->_backendId.isEmpty())
            return;
        if (!queryIsEmpty()) {
            // setup notifications
            QJsonObject filter;
            QJsonObject objectType;
            objectType.insert(EnginioString::objectType, queryData(EnginioString::objectType));
            filter.insert(EnginioString::data, objectType);
            _notifications.connectToBackend(this, _enginio, filter);

            EnginioReplyState *ereply = reload();
            QObject::connect(ereply, &EnginioReplyState::dataChanged, ereply, &EnginioReplyState::deleteLater);
        } else {
            fullQueryReset(QJsonArray());
        }
    }

    EnginioReplyState *reload()
    {
        // send full query
        QJsonObject query = queryAsJson();
        ObjectAdaptor<QJsonObject> aQuery(query);
        QNetworkReply *nreply = _enginio->query(aQuery, static_cast<Enginio::Operation>(_operation));
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        if (_canFetchMore)
            _latestRequestedOffset = query[EnginioString::limit].toDouble();
        FinishedFullQueryRequest finshedRequest = { this, ereply };
        QObject::connect(ereply, &EnginioReplyState::dataChanged, _replyConnectionConntext, finshedRequest);
        return ereply;
    }

    void finishedIncrementalUpdateRequest(const EnginioReplyState *reply, const QJsonObject &query)
    {
        Q_ASSERT(_canFetchMore);
        QJsonArray data(replyData(reply)[EnginioString::results].toArray());
        int offset = query[EnginioString::offset].toDouble();
        int limit = query[EnginioString::limit].toDouble();
        int dataCount = data.count();

        int startingOffset = qMax(offset, _data.count());

        q->beginInsertRows(QModelIndex(), startingOffset, startingOffset + dataCount -1);
        for (int i = 0; i < dataCount; ++i) {
            _data.append(data[i]);
        }

        _canFetchMore = limit <= dataCount;
        q->endInsertRows();
    }

    void finishedFullQueryRequest(const EnginioReplyState *reply)
    {
        delete _replyConnectionConntext;
        _replyConnectionConntext = new QObject();
        fullQueryReset(replyData(reply)[EnginioString::results].toArray());
    }

    void fullQueryReset(const QJsonArray &data);

    void finishedCreateRequest(const EnginioReplyState *reply, const QString &tmpId)
    {
        if (_attachedData.markRequestIdAsHandled(reply->requestId()))
            return; // request was handled

        int row;
        if (_attachedData.contains(tmpId))
            // this is a common path, we got result of our create request and we still have a dummy
            // item that we want to update.
            row = _attachedData.deref(tmpId).row;
        else {
            // the dummy object doesn't exist anymore, probably it was removed by a full reset
            // or by an initial query.
            QString id = replyData(reply)[EnginioString::id].toString();
            if (_attachedData.contains(id)) {
                // The reset removed the dummy value but it contained the newly created (initial reset
                // and append were reordered)
                row = _attachedData.rowFromObjectId(id);
            } else {
                // we created the item but there is no sign of it. We need to check if we have more or
                // less the same query, there is a chance that the value was lost in race between the
                // reset and create. This is possible scenario that:
                // send create -> send full query -> do query -> do create -> got query -> got create
                if (queryData(EnginioString::objectType) == replyData(reply)[EnginioString::objectType]) {
                    // the type is the same so we can re-add it
                    receivedCreateNotification(replyData(reply));
                }
                // query was changed too much we are done.
                return;
            }
        }

        if (reply->networkError() != QNetworkReply::NoError) {
            // We tried to create something and we failed, we need to remove tmp
            // item

            // TODO add a signal here so a developer can ask an user for a conflict
            // resolution.
            receivedRemoveNotification(_data[row].toObject(), row);
            return;
        }

        const QJsonObject object = replyData(reply);
        receivedUpdateNotification(object, tmpId, row);
    }

    void finishedRemoveRequest(const EnginioReplyState *response, const QString &id)
    {
        if (!_attachedData.contains(id))
            return; // we do not know the object anymore, we are not interested in it's delete event

        AttachedData &data = _attachedData.deref(id);

        if (_attachedData.markRequestIdAsHandled(response->requestId()))
            return; // request was handled


        int row = data.row;
        if (row == DeletedRow || (response->networkError() != QNetworkReply::NoError && response->backendStatus() != 404)) {
            if (!data.ref) {
                // The item was not removed, because of an error. We assume that the
                // item is in sync
                emit q->dataChanged(q->index(row), q->index(row));
            }
            return;
        }
        receivedRemoveNotification(_data[row].toObject(), row);
    }

    void finishedUpdateRequest(const EnginioReplyState *reply, const QString &id, const QJsonObject &oldValue)
    {
        AttachedData &data = _attachedData.deref(id);

        if (_attachedData.markRequestIdAsHandled(reply->requestId()))
            return; // request was handled

        int row = data.row;
        if (row == DeletedRow) {
            // We tried to update something that we already deleted
            // everything should be handled
            return;
        }
        if (reply->networkError() != QNetworkReply::NoError) {
            if (reply->backendStatus() == 404) {
                // We tried to update something that got deleted in between, probably on
                // the server side. Changing operation type to remove, so the cache
                // can be in sync with the server again.

                // TODO add a signal here so a developer can ask an user for a conflict
                // resolution.
                receivedRemoveNotification(_data[row].toObject(), row);
            } else {
                // Try to rollback the change.
                // TODO it is not perfect https://github.com/enginio/enginio-qt/issues/200
                _data.replace(row, oldValue);
                emit q->dataChanged(q->index(row), q->index(row));
            }
            return;
        }
        receivedUpdateNotification(replyData(reply), id, row);
    }

    struct SwapNetworkReplyForSetData
    {
        SwapNetworkReplyBase d;
        QVariant _value;
        int _role;
        EnginioReplyState *finishedCreateReply;

        void operator ()()
        {

            if (finishedCreateReply->isError()) {
                d.markAsError(EnginioString::Dependent_create_query_failed_so_object_could_not_be_updated);
            } else if (Q_UNLIKELY(!d._modelGuard)) {
                d.markAsError(EnginioString::EnginioModel_was_removed_before_this_request_was_prepared);
            } else {
                QPair<QString, int> tmp = d.getAndSetCurrentIdRow(finishedCreateReply);
                const int row = tmp.second;
                if (Q_UNLIKELY(row == InvalidRow)) {
                    d.markAsError(EnginioString::EnginioModel_The_query_was_changed_before_the_request_could_be_sent);
                    return;
                }
                QString id = tmp.first;
                FinishedUpdateRequest finished = { d._model, id, d._object, d._reply };
                QObject::connect(d._reply, &EnginioReplyState::dataChanged, d._model->_replyConnectionConntext, finished);
                EnginioReplyState *ereply = d._model->setDataNow(row, _value, _role, d._object, id);
                d.swapNetworkReply(ereply);
            }
        }
    };

    EnginioReplyState *setData(const int row, const QVariant &value, int role)
    {
        if (role != Enginio::InvalidRole) {
            QJsonObject oldObject = _data.at(row).toObject();
            QString id = oldObject[EnginioString::id].toString();
            if (id.isEmpty())
                return setDataDelyed(row, value, role, oldObject);
            return setDataNow(row, value, role, oldObject, id);
        }
        QNetworkReply *nreply = new EnginioFakeReply(_enginio, EnginioClientConnectionPrivate::constructErrorMessage(EnginioString::EnginioModel_Trying_to_update_an_object_with_unknown_role));
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        return ereply;
    }

    void delayedOperation(int row, EnginioReplyState **newReply, QString *tmpId, EnginioReplyState **createReply)
    {
        Q_ASSERT(!_attachedData.isSynced(row));
        AttachedData data = _attachedData.ref(row);
        *createReply = data.createReply;
        Q_ASSERT(*createReply);
        *tmpId = data.id;
        Q_ASSERT(tmpId->startsWith('t'));
        EnginioDummyReply *nreply = new EnginioDummyReply(*createReply);
        *newReply = _enginio->createReply(nreply);
    }

    EnginioReplyState *setDataDelyed(int row, const QVariant &value, int role, const QJsonObject &oldObject)
    {
        // We are about to update a not synced new item. The item do not have id yet,
        // so we can not make a request now, we need to wait for finished signal.
        Q_ASSERT(role > Enginio::SyncedRole);
        EnginioReplyState *ereply, *createReply;
        QString tmpId;
        Q_ASSERT(oldObject[EnginioString::id].toString().isEmpty());
        delayedOperation(row, &ereply, &tmpId, &createReply);
        SwapNetworkReplyForSetData swapNetworkReply = {{ereply, this, oldObject, tmpId, q}, value, role, createReply};
        QObject::connect(createReply, &EnginioReplyState::dataChanged, swapNetworkReply);
        return ereply;
    }

    EnginioReplyState *setDataNow(const int row, const QVariant &value, int role, const QJsonObject &oldObject, const QString &id)
    {
        Q_ASSERT(!id.isEmpty());
        QJsonObject deltaObject;
        QJsonObject newObject = oldObject;
        if (role != Enginio::JsonObjectRole) {
            const QString roleName(_roles.value(role));
            Q_ASSERT(!roleName.isEmpty());
            deltaObject[roleName] = newObject[roleName] = QJsonValue::fromVariant(value);
        } else {
            const QJsonObject updateObject = value.toJsonObject();
            if (updateObject.isEmpty()) {
                QNetworkReply *nreply = new EnginioFakeReply(_enginio, EnginioClientConnectionPrivate::constructErrorMessage(EnginioString::EnginioModel_Trying_to_update_an_object_with_unknown_role));
                return _enginio->createReply(nreply);
            }
            for (QJsonObject::const_iterator i = updateObject.constBegin(); i != updateObject.constEnd(); ++i)
                deltaObject[i.key()] = i.value();
        }
        deltaObject[EnginioString::id] = id;
        deltaObject[EnginioString::objectType] = newObject[EnginioString::objectType];
        ObjectAdaptor<QJsonObject> aDeltaObject(deltaObject);
        QNetworkReply *nreply = _enginio->update(aDeltaObject, _operation);
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        FinishedUpdateRequest finished = { this, id, oldObject, ereply };
        QObject::connect(ereply, &EnginioReplyState::dataChanged, _replyConnectionConntext, finished);
        _attachedData.ref(id, row);
        _data.replace(row, newObject);
        _attachedData.insertRequestId(ereply->requestId(), row);
        emit q->dataChanged(q->index(row), q->index(row));
        return ereply;
    }

    void syncRoles();

    QHash<int, QByteArray> roleNames() const Q_REQUIRED_RESULT
    {
        QHash<int, QByteArray> roles;
        roles.reserve(_roles.count());
        for (QHash<int, QString>::const_iterator i = _roles.constBegin();
             i != _roles.constEnd();
             ++i) {
            roles.insert(i.key(), i.value().toUtf8());
        }
        return roles;
    }

    int rowCount() const Q_REQUIRED_RESULT
    {
        return _data.count();
    }

    QVariant data(unsigned row, int role) const Q_REQUIRED_RESULT
    {
        if (role == Enginio::SyncedRole) {
            return _attachedData.isSynced(row);
        }

        const QJsonObject object = _data.at(row).toObject();
        if (!object.isEmpty()) {
            if (role == Qt::DisplayRole || role == Enginio::JsonObjectRole)
                return _data.at(row);
            const QString roleName = _roles.value(role);
            if (!roleName.isEmpty())
                return object[roleName];
        }

        return QVariant();
    }

    bool canFetchMore() const Q_REQUIRED_RESULT
    {
        return _canFetchMore;
    }

    void fetchMore(int row)
    {
        int currentOffset = _data.count();
        if (!_canFetchMore || currentOffset < _latestRequestedOffset)
            return; // we do not want to spam the server, lets wait for the last fetch

        QJsonObject query(queryAsJson());

        int limit = query[EnginioString::limit].toDouble();
        limit = qMax(row - currentOffset, limit); // check if default limit is not too small

        query[EnginioString::offset] = currentOffset;
        query[EnginioString::limit] = limit;

        qDebug() << Q_FUNC_INFO << query;
        _latestRequestedOffset += limit;
        ObjectAdaptor<QJsonObject> aQuery(query);
        QNetworkReply *nreply = _enginio->query(aQuery, static_cast<Enginio::Operation>(_operation));
        EnginioReplyState *ereply = _enginio->createReply(nreply);
        QObject::connect(ereply, &EnginioReplyState::dataChanged, ereply, &EnginioReplyState::deleteLater);
        FinishedIncrementalUpdateRequest finishedRequest = { this, query, ereply };
        QObject::connect(ereply, &EnginioReplyState::dataChanged, _replyConnectionConntext, finishedRequest);
    }

    virtual QJsonObject replyData(const EnginioReplyState *reply) const = 0;
    virtual QJsonValue queryData(const QString &name) = 0;
    virtual bool queryIsEmpty() const = 0;
    virtual QJsonObject queryAsJson() const = 0;
};


template<typename Derived, typename Types>
struct EnginioModelPrivateT : public EnginioBaseModelPrivate
{
    typedef EnginioBaseModelPrivate Base;
    typedef typename Types::Reply Reply;
    typedef typename Types::Public Public;
    typedef typename Types::Client Client;
    typedef typename Types::ClientPrivate ClientPrivate;
    typedef typename Types::Data Data;

    Data _query;

    inline Public *q() const { return static_cast<Public*>(Base::q); }

    class EnginioDestroyed
    {
        EnginioModelPrivateT *model;
    public:
        EnginioDestroyed(EnginioModelPrivateT *m)
            : model(m)
        {
            Q_ASSERT(m);
        }
        void operator ()()
        {
            model->setClient(0);
        }
    };

    EnginioModelPrivateT(EnginioBaseModel *pub)
        : Base(pub)
    {}

    void init()
    {
        QObject::connect(q(), &Public::queryChanged, QueryChanged(this));
        QObject::connect(q(), &Public::clientChanged, QueryChanged(this));
        QObject::connect(q(), &Public::operationChanged, QueryChanged(this));
    }

    Client *enginio() const Q_REQUIRED_RESULT
    {
        return _enginio ? ClientPrivate::get(_enginio) : 0;
    }

    void setClient(const EnginioClientConnection *enginio)
    {
        if (_enginio) {
            foreach (const QMetaObject::Connection &connection, _clientConnections)
                QObject::disconnect(connection);
            _clientConnections.clear();
        }
        if (enginio) {
            _enginio = EnginioClientConnectionPrivate::get(const_cast<EnginioClientConnection*>(enginio));
            _clientConnections.append(QObject::connect(enginio, &QObject::destroyed, EnginioDestroyed(this)));
            _clientConnections.append(QObject::connect(enginio, &EnginioClientConnection::backendIdChanged, QueryChanged(this)));
            _clientConnections.append(QObject::connect(enginio, &EnginioClientConnection::authenticationStateChanged, RefreshQueryAfterAuthChange(this)));
        } else {
            _enginio = 0;
        }

        q()->clientChanged(static_cast<Client*>(const_cast<EnginioClientConnection*>(enginio)));
    }

    Data query() Q_REQUIRED_RESULT
    {
        return _query;
    }

    void setQuery(const Data &query)
    {
        _query = query;

        // TODO Enable together with pageing support
//        if (_query.contains(EnginioString::pageSize)) {
//            const int pageSize = _query[EnginioString::pageSize].toDouble();
//            const QString limitString(EnginioString::limit);
//            const QString offsetString(EnginioString::offset);
//            const unsigned limit = _query[limitString].toDouble();
//            const unsigned offset = _query[offsetString].toDouble();
//            if (limit)
//                qWarning() << "EnginioModel::setQuery()" << "'limit' parameter can not be used together with model pagining feature, the value will be ignored";

//            if (offset) {
//                qWarning() << "EnginioModel::setQuery()" << "'offset' parameter can not be used together with model pagining feature, the value will be ignored";
//                _query.remove(offsetString);
//            }
//            _query[limitString] = pageSize;
//            _canFetchMore = true;
//        } else {
//            _canFetchMore = false;
//        }
        emit q()->queryChanged(query);
    }

    Reply *append(const QJsonObject &value) { return static_cast<Reply*>(Base::append(value)); }
    Reply *remove(int row) { return static_cast<Reply*>(Base::remove(row)); }
    Reply *setValue(int row, const QString &role, const QVariant &value) { return static_cast<Reply*>(Base::setValue(row, role, value)); }
    Reply *reload() { return static_cast<Reply*>(Base::reload()); }
    Reply *setData(const int row, const QVariant &value, int role) { return static_cast<Reply*>(Base::setData(row, value, role)); }
    bool queryIsEmpty() const Q_DECL_OVERRIDE
    {
        return ObjectAdaptor<Data>(_query, static_cast<ClientPrivate*>(_enginio)).isEmpty();
    }
};

QT_END_NAMESPACE
#endif // ENGINIOMODELBASE_P_H
