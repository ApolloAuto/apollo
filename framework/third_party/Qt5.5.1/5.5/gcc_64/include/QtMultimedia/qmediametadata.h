/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Toolkit.
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

#ifndef QMEDIAMETADATA_H
#define QMEDIAMETADATA_H

#include <QtCore/qpair.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qstring.h>

#include <QtMultimedia/qtmultimediadefs.h>

QT_BEGIN_NAMESPACE

// Class forward declaration required for QDoc bug
class QString;

#define Q_DECLARE_METADATA(key) Q_MULTIMEDIA_EXPORT extern const QString key

namespace QMediaMetaData {
#ifdef Q_QDOC
    // QDoc does not like macros, so try to keep this in sync :)
    QString Title;
    QString SubTitle;
    QString Author;
    QString Comment;
    QString Description;
    QString Category;
    QString Genre;
    QString Year;
    QString Date;
    QString UserRating;
    QString Keywords;
    QString Language;
    QString Publisher;
    QString Copyright;
    QString ParentalRating;
    QString RatingOrganization;

    // Media
    QString Size;
    QString MediaType;
    QString Duration;

    // Audio
    QString AudioBitRate;
    QString AudioCodec;
    QString AverageLevel;
    QString ChannelCount;
    QString PeakValue;
    QString SampleRate;

    // Music
    QString AlbumTitle;
    QString AlbumArtist;
    QString ContributingArtist;
    QString Composer;
    QString Conductor;
    QString Lyrics;
    QString Mood;
    QString TrackNumber;
    QString TrackCount;

    QString CoverArtUrlSmall;
    QString CoverArtUrlLarge;

    // Image/Video
    QString Resolution;
    QString PixelAspectRatio;

    // Video
    QString VideoFrameRate;
    QString VideoBitRate;
    QString VideoCodec;

    QString PosterUrl;

    // Movie
    QString ChapterNumber;
    QString Director;
    QString LeadPerformer;
    QString Writer;

    // Photos
    QString CameraManufacturer;
    QString CameraModel;
    QString Event;
    QString Subject;
    QString Orientation;
    QString ExposureTime;
    QString FNumber;
    QString ExposureProgram;
    QString ISOSpeedRatings;
    QString ExposureBiasValue;
    QString DateTimeOriginal;
    QString DateTimeDigitized;
    QString SubjectDistance;
    QString MeteringMode;
    QString LightSource;
    QString Flash;
    QString FocalLength;
    QString ExposureMode;
    QString WhiteBalance;
    QString DigitalZoomRatio;
    QString FocalLengthIn35mmFilm;
    QString SceneCaptureType;
    QString GainControl;
    QString Contrast;
    QString Saturation;
    QString Sharpness;
    QString DeviceSettingDescription;

    // Location
    QString GPSLatitude;
    QString GPSLongitude;
    QString GPSAltitude;
    QString GPSTimeStamp;
    QString GPSSatellites;
    QString GPSStatus;
    QString GPSDOP;
    QString GPSSpeed;
    QString GPSTrack;
    QString GPSTrackRef;
    QString GPSImgDirection;
    QString GPSImgDirectionRef;
    QString GPSMapDatum;
    QString GPSProcessingMethod;
    QString GPSAreaInformation;

    QString PosterImage;
    QString CoverArtImage;
    QString ThumbnailImage;
#else
    // Common
    Q_DECLARE_METADATA(Title);
    Q_DECLARE_METADATA(SubTitle);
    Q_DECLARE_METADATA(Author);
    Q_DECLARE_METADATA(Comment);
    Q_DECLARE_METADATA(Description);
    Q_DECLARE_METADATA(Category);
    Q_DECLARE_METADATA(Genre);
    Q_DECLARE_METADATA(Year);
    Q_DECLARE_METADATA(Date);
    Q_DECLARE_METADATA(UserRating);
    Q_DECLARE_METADATA(Keywords);
    Q_DECLARE_METADATA(Language);
    Q_DECLARE_METADATA(Publisher);
    Q_DECLARE_METADATA(Copyright);
    Q_DECLARE_METADATA(ParentalRating);
    Q_DECLARE_METADATA(RatingOrganization);

    // Media
    Q_DECLARE_METADATA(Size);
    Q_DECLARE_METADATA(MediaType);
    Q_DECLARE_METADATA(Duration);

    // Audio
    Q_DECLARE_METADATA(AudioBitRate);
    Q_DECLARE_METADATA(AudioCodec);
    Q_DECLARE_METADATA(AverageLevel);
    Q_DECLARE_METADATA(ChannelCount);
    Q_DECLARE_METADATA(PeakValue);
    Q_DECLARE_METADATA(SampleRate);

    // Music
    Q_DECLARE_METADATA(AlbumTitle);
    Q_DECLARE_METADATA(AlbumArtist);
    Q_DECLARE_METADATA(ContributingArtist);
    Q_DECLARE_METADATA(Composer);
    Q_DECLARE_METADATA(Conductor);
    Q_DECLARE_METADATA(Lyrics);
    Q_DECLARE_METADATA(Mood);
    Q_DECLARE_METADATA(TrackNumber);
    Q_DECLARE_METADATA(TrackCount);

    Q_DECLARE_METADATA(CoverArtUrlSmall);
    Q_DECLARE_METADATA(CoverArtUrlLarge);

    // Image/Video
    Q_DECLARE_METADATA(Resolution);
    Q_DECLARE_METADATA(PixelAspectRatio);

    // Video
    Q_DECLARE_METADATA(VideoFrameRate);
    Q_DECLARE_METADATA(VideoBitRate);
    Q_DECLARE_METADATA(VideoCodec);

    Q_DECLARE_METADATA(PosterUrl);

    // Movie
    Q_DECLARE_METADATA(ChapterNumber);
    Q_DECLARE_METADATA(Director);
    Q_DECLARE_METADATA(LeadPerformer);
    Q_DECLARE_METADATA(Writer);

    // Photos
    Q_DECLARE_METADATA(CameraManufacturer);
    Q_DECLARE_METADATA(CameraModel);
    Q_DECLARE_METADATA(Event);
    Q_DECLARE_METADATA(Subject);
    Q_DECLARE_METADATA(Orientation);
    Q_DECLARE_METADATA(ExposureTime);
    Q_DECLARE_METADATA(FNumber);
    Q_DECLARE_METADATA(ExposureProgram);
    Q_DECLARE_METADATA(ISOSpeedRatings);
    Q_DECLARE_METADATA(ExposureBiasValue);
    Q_DECLARE_METADATA(DateTimeOriginal);
    Q_DECLARE_METADATA(DateTimeDigitized);
    Q_DECLARE_METADATA(SubjectDistance);
    Q_DECLARE_METADATA(MeteringMode);
    Q_DECLARE_METADATA(LightSource);
    Q_DECLARE_METADATA(Flash);
    Q_DECLARE_METADATA(FocalLength);
    Q_DECLARE_METADATA(ExposureMode);
    Q_DECLARE_METADATA(WhiteBalance);
    Q_DECLARE_METADATA(DigitalZoomRatio);
    Q_DECLARE_METADATA(FocalLengthIn35mmFilm);
    Q_DECLARE_METADATA(SceneCaptureType);
    Q_DECLARE_METADATA(GainControl);
    Q_DECLARE_METADATA(Contrast);
    Q_DECLARE_METADATA(Saturation);
    Q_DECLARE_METADATA(Sharpness);
    Q_DECLARE_METADATA(DeviceSettingDescription);

    // Location
    Q_DECLARE_METADATA(GPSLatitude);
    Q_DECLARE_METADATA(GPSLongitude);
    Q_DECLARE_METADATA(GPSAltitude);
    Q_DECLARE_METADATA(GPSTimeStamp);
    Q_DECLARE_METADATA(GPSSatellites);
    Q_DECLARE_METADATA(GPSStatus);
    Q_DECLARE_METADATA(GPSDOP);
    Q_DECLARE_METADATA(GPSSpeed);
    Q_DECLARE_METADATA(GPSTrack);
    Q_DECLARE_METADATA(GPSTrackRef);
    Q_DECLARE_METADATA(GPSImgDirection);
    Q_DECLARE_METADATA(GPSImgDirectionRef);
    Q_DECLARE_METADATA(GPSMapDatum);
    Q_DECLARE_METADATA(GPSProcessingMethod);
    Q_DECLARE_METADATA(GPSAreaInformation);

    Q_DECLARE_METADATA(PosterImage);
    Q_DECLARE_METADATA(CoverArtImage);
    Q_DECLARE_METADATA(ThumbnailImage);
#endif
}

#undef Q_DECLARE_METADATA

QT_END_NAMESPACE

#endif // QMEDIAMETADATA_H
